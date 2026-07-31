#include "auto_coreball/image_processor.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <algorithm>

ImageProcessor::ImageProcessor()
    : rotation_radius_(0.0), radius_learned_(false) {}

void ImageProcessor::PreprocessFrame(const cv::Mat& input, cv::Mat& output) {
  if (input.channels() > 1) {
    cv::cvtColor(input, output, cv::COLOR_BGR2GRAY);
  } else {
    output = input.clone();
  }
}

bool ImageProcessor::DetectDisc(const cv::Mat& gray, cv::Point2f& center,
                                double& radius) {
  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(11, 11), 3);

  cv::Mat binary;
  cv::threshold(blurred, binary, 180, 255, cv::THRESH_BINARY);

  cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(5, 5));
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel_open);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  double best_area = 0;
  int best_idx = -1;
  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area > best_area) {
      best_area = area;
      best_idx = static_cast<int>(i);
    }
  }

  if (best_idx < 0 || best_area < 500) {
    return false;
  }

  cv::Point2f mc;
  float mr;
  cv::minEnclosingCircle(contours[best_idx], mc, mr);

  double peri = cv::arcLength(contours[best_idx], true);
  double circularity = (peri > 0) ? 4 * CV_PI * best_area / (peri * peri) : 0;
  double area_ratio = (mr > 0) ? best_area / (CV_PI * mr * mr) : 0;

  if (circularity < 0.7 || area_ratio < 0.6 || mr < 30 || mr > 200) {
    return false;
  }

  center = mc;
  radius = mr;

  return true;
}

double ImageProcessor::ComputeAngle(const cv::Point2f& pt,
                                    const cv::Point2f& center) {
  cv::Point2f dir = pt - center;
  return std::atan2(dir.y, dir.x);
}

std::vector<NumberedBall> ImageProcessor::DetectNumberedBalls(
    const cv::Mat& gray, const cv::Point2f& center, double disc_radius) {
  std::vector<NumberedBall> balls;

  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(11, 11), 3);

  cv::Mat binary;
  cv::threshold(blurred, binary, 180, 255, cv::THRESH_BINARY);

  cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(5, 5));
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel_open);

  cv::Mat disc_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
  cv::circle(disc_mask, center, static_cast<int>(disc_radius * 1.15),
             cv::Scalar(255), -1);
  cv::bitwise_and(binary, ~disc_mask, binary);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary.clone(), contours, hierarchy, cv::RETR_CCOMP,
                   cv::CHAIN_APPROX_SIMPLE);

  for (size_t ci = 0; ci < contours.size(); ++ci) {
    const auto& contour = contours[ci];
    double area = cv::contourArea(contour);
    cv::Point2f mc;
    float mr;
    cv::minEnclosingCircle(contour, mc, mr);
    double dist = cv::norm(mc - center);
    double peri = cv::arcLength(contour, true);
    double circularity = (peri > 0) ? 4 * CV_PI * area / (peri * peri) : 0;

    if (area < 80) continue;

    if (mr < disc_radius * 0.03 || mr > disc_radius * 0.5) continue;

    if (dist < disc_radius * 0.7 || dist > disc_radius * 7.0) continue;

    {
      double max_y = radius_learned_
          ? (center.y + rotation_radius_ * 1.5)
          : (center.y + disc_radius * 3.5);
      if (mc.y > max_y) continue;
    }

    if (circularity < 0.2) continue;

    if (hierarchy[ci][3] != -1) continue;

    NumberedBall ball;
    ball.center = mc;
    ball.radius = mr;
    ball.angle_rad = ComputeAngle(mc, center);
    ball.number = 0;
    ball.has_number = (hierarchy[ci][2] != -1);
    balls.push_back(ball);
  }

  if (!radius_learned_) {
    if (!ball_history_.empty()) {
      const auto& prev = ball_history_.back();
      for (const auto& b : balls) {
        double best_diff = 1000.0;
        for (const auto& pb : prev) {
          double diff = b.angle_rad - pb.angle_rad;
          while (diff > CV_PI) diff -= 2 * CV_PI;
          while (diff < -CV_PI) diff += 2 * CV_PI;
          if (std::abs(diff) < std::abs(best_diff))
            best_diff = diff;
        }
        if (std::abs(best_diff) > 0.01 && std::abs(best_diff) < 0.3) {
          rotation_samples_.push_back(cv::norm(b.center - center));
        }
      }
    }
    ball_history_.push_back(balls);
    if (ball_history_.size() > 10) ball_history_.pop_front();
    if (rotation_samples_.size() >= 15) {
      double sum = 0;
      for (auto d : rotation_samples_) sum += d;
      rotation_radius_ = sum / rotation_samples_.size();
      radius_learned_ = true;
    }
  } else {
    double min_dist = rotation_radius_ * 0.7;
    double max_dist = rotation_radius_ * 1.3;
    std::vector<NumberedBall> filtered;
    for (const auto& b : balls) {
      double dist = cv::norm(b.center - center);
      if (dist >= min_dist && dist <= max_dist)
        filtered.push_back(b);
    }
    balls = filtered;
  }

  auto angle_cmp = [](const NumberedBall& a, const NumberedBall& b) {
    return a.angle_rad < b.angle_rad;
  };
  std::sort(balls.begin(), balls.end(), angle_cmp);

  std::vector<NumberedBall> deduped;
  for (size_t i = 0; i < balls.size(); ++i) {
    if (i == 0 || std::abs(balls[i].angle_rad - balls[i-1].angle_rad) > 0.017) {
      deduped.push_back(balls[i]);
    }
  }
  balls = deduped;

  if (balls.size() >= 2) {
    double wrap_diff = balls.front().angle_rad + 2 * CV_PI - balls.back().angle_rad;
    if (wrap_diff < 0.017)
      balls.pop_back();
  }

  return balls;
}

std::vector<Pin> ImageProcessor::DetectPins(const cv::Mat& gray,
                                            const cv::Point2f& center,
                                            double radius) {
  std::vector<Pin> pins;

  auto balls = DetectNumberedBalls(gray, center, radius);

  for (size_t i = 0; i < balls.size(); ++i) {
    Pin pin;
    pin.id = static_cast<int>(i);
    pin.angle_rad = balls[i].angle_rad;
    pin.inner_pt = center + (balls[i].center - center) *
                             (radius / cv::norm(balls[i].center - center));
    pin.outer_pt = balls[i].center;
    pin.length = cv::norm(balls[i].center - pin.inner_pt);
    pin.has_number = balls[i].has_number;
    pins.push_back(pin);
  }

  auto angle_cmp = [](const Pin& a, const Pin& b) {
    return a.angle_rad < b.angle_rad;
  };
  std::sort(pins.begin(), pins.end(), angle_cmp);

  return pins;
}
