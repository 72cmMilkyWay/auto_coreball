#include "auto_coreball/image_processor.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

ImageProcessor::ImageProcessor()
    : debug_draw_(true), roi_ratio_(0.7),
      rotation_radius_(0.0), frames_since_detection_(0), radius_learned_(false) {}

cv::Rect ImageProcessor::GetCenterROI(int width, int height) const {
  int roi_w = static_cast<int>(width * roi_ratio_);
  int roi_h = static_cast<int>(height * roi_ratio_);
  int x = (width - roi_w) / 2;
  int y = (height - roi_h) / 2;
  return cv::Rect(x, y, roi_w, roi_h);
}

void ImageProcessor::PreprocessFrame(const cv::Mat& input, cv::Mat& output) {
  if (input.channels() > 1) {
    cv::cvtColor(input, output, cv::COLOR_BGR2GRAY);
  } else {
    output = input.clone();
  }
}

bool ImageProcessor::DetectDisc(const cv::Mat& gray, cv::Point2f& center,
                                double& radius) {
  if (debug_draw_) {
    gray.copyTo(debug_img_);
    cv::cvtColor(debug_img_, debug_img_, cv::COLOR_GRAY2BGR);
  }

  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(11, 11), 3);

  cv::Mat binary;
  cv::threshold(blurred, binary, 180, 255, cv::THRESH_BINARY);

  cv::imwrite("debug_01_binary.png", binary);

  cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(5, 5));
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel_open);

  cv::imwrite("debug_02_after_open.png", binary);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  cv::Mat contour_img = cv::Mat::zeros(gray.size(), CV_8UC3);
  for (size_t i = 0; i < contours.size(); ++i) {
    cv::Scalar color(0, 255, 255);
    cv::drawContours(contour_img, contours, static_cast<int>(i), color, 2);
  }
  cv::imwrite("debug_04_contours.png", contour_img);

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
    if (debug_draw_) {
      cv::putText(debug_img_, "No disc found", cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    return false;
  }

  cv::Point2f mc;
  float mr;
  cv::minEnclosingCircle(contours[best_idx], mc, mr);

  double peri = cv::arcLength(contours[best_idx], true);
  double circularity = (peri > 0) ? 4 * CV_PI * best_area / (peri * peri) : 0;
  double area_ratio = (mr > 0) ? best_area / (CV_PI * mr * mr) : 0;

  if (circularity < 0.7 || area_ratio < 0.6 || mr < 30 || mr > 200) {
    if (debug_draw_) {
      cv::putText(debug_img_, "Disc shape bad", cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    return false;
  }

  center = mc;
  radius = mr;

  cv::Mat result = gray.clone();
  cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
  cv::circle(result, center, static_cast<int>(radius),
             cv::Scalar(0, 255, 0), 2);
  cv::circle(result, center, 4, cv::Scalar(0, 0, 255), -1);
  cv::imwrite("debug_05_result.png", result);

  if (debug_draw_) {
    cv::circle(debug_img_, center, static_cast<int>(radius),
               cv::Scalar(0, 255, 0), 2);
    cv::circle(debug_img_, center, 4, cv::Scalar(0, 0, 255), -1);
  }

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

  cv::imwrite("debug_06a_before_open.png", binary);

  cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(5, 5));
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel_open);

  cv::Mat disc_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
  cv::circle(disc_mask, center, static_cast<int>(disc_radius * 1.15),
             cv::Scalar(255), -1);
  cv::bitwise_and(binary, ~disc_mask, binary);

  cv::imwrite("debug_06_balls_binary.png", binary);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary.clone(), contours, hierarchy, cv::RETR_CCOMP,
                   cv::CHAIN_APPROX_SIMPLE);

  cv::Mat ball_contour_img = cv::Mat::zeros(gray.size(), CV_8UC3);
  for (size_t i = 0; i < contours.size(); ++i) {
    cv::Scalar color(255, 0, 255);
    cv::drawContours(ball_contour_img, contours, static_cast<int>(i), color, 2);
  }
  cv::imwrite("debug_07_ball_contours.png", ball_contour_img);

  double disc_area = CV_PI * disc_radius * disc_radius;

  std::cout << "[Pins] contours=" << contours.size() << " disc_r="
            << static_cast<int>(disc_radius) << std::endl;

  int rejected_area = 0, rejected_r = 0, rejected_dist = 0, rejected_circ = 0;

  for (size_t ci = 0; ci < contours.size(); ++ci) {
    const auto& contour = contours[ci];
    double area = cv::contourArea(contour);
    cv::Point2f mc;
    float mr;
    cv::minEnclosingCircle(contour, mc, mr);
    double dist = cv::norm(mc - center);
    double peri = cv::arcLength(contour, true);
    double circularity = (peri > 0) ? 4 * CV_PI * area / (peri * peri) : 0;

    if (area < 80) { rejected_area++; continue; }

    if (mr < disc_radius * 0.03 || mr > disc_radius * 0.5) { rejected_r++; continue; }

    if (dist < disc_radius * 0.7 || dist > disc_radius * 7.0) { rejected_dist++; continue; }

    // 排除下方等待球：圆盘下方远处的球不可能是针
    {
      double max_y = radius_learned_
          ? (center.y + rotation_radius_ * 1.5)
          : (center.y + disc_radius * 3.5);
      if (mc.y > max_y) { rejected_dist++; continue; }
    }

    if (circularity < 0.2) { rejected_circ++; continue; }

    NumberedBall ball;
    ball.center = mc;
    ball.radius = mr;
    ball.angle_rad = ComputeAngle(mc, center);
    ball.number = 0;
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
      std::cout << "[Learn] rotation_radius=" << rotation_radius_ << std::endl;
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

  // 去重：角度差 < 1° 的视为同一个针
  std::vector<NumberedBall> deduped;
  for (size_t i = 0; i < balls.size(); ++i) {
    if (i == 0 || std::abs(balls[i].angle_rad - balls[i-1].angle_rad) > 0.017) {
      deduped.push_back(balls[i]);
    }
  }
  balls = deduped;

  std::cout << "[Pins] accepted=" << balls.size()
            << " rejected: area=" << rejected_area
            << " r=" << rejected_r
            << " dist=" << rejected_dist
            << " circ=" << rejected_circ << std::endl;

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
    pins.push_back(pin);
  }

  auto angle_cmp = [](const Pin& a, const Pin& b) {
    return a.angle_rad < b.angle_rad;
  };
  std::sort(pins.begin(), pins.end(), angle_cmp);

  if (debug_draw_) {
    for (const auto& pin : pins) {
      cv::line(debug_img_, pin.inner_pt, pin.outer_pt,
               cv::Scalar(255, 0, 0), 2);
      cv::circle(debug_img_, pin.outer_pt, 4, cv::Scalar(0, 255, 255), -1);
    }
  }

  return pins;
}

bool ImageProcessor::IsLineThroughCircle(const cv::Vec4i& line,
                                         const cv::Point2f& center,
                                         double radius, double tolerance) {
  (void)tolerance;
  cv::Point2f p1(line[0], line[1]);
  cv::Point2f p2(line[2], line[3]);
  cv::Point2f mid = (p1 + p2) * 0.5f;
  double dist = cv::norm(mid - center);
  return dist < radius * 0.3;
}

double ImageProcessor::ComputeLineAngle(const cv::Vec4i& line,
                                        const cv::Point2f& center) {
  cv::Point2f p1(line[0], line[1]);
  cv::Point2f p2(line[2], line[3]);
  cv::Point2f mid = (p1 + p2) * 0.5f;
  cv::Point2f dir = mid - center;
  return std::atan2(dir.y, dir.x);
}

cv::Point2f ImageProcessor::FindInsertionPoint(const cv::Mat& gray,
                                               const cv::Point2f& center,
                                               double radius, bool& found) {
  (void)gray;
  found = true;
  return cv::Point2f(center.x, center.y + radius + 30);
}

void ImageProcessor::SaveDebugImage(const std::string& path) const {
  if (!debug_img_.empty()) {
    cv::imwrite(path, debug_img_);
  }
}
