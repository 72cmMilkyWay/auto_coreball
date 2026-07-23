#include "auto_coreball/motion_estimator.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

MotionEstimator::MotionEstimator()
    : initialized_(false),
      filtered_angle_(0.0),
      angular_velocity_(0.0),
      velocity_confidence_(0.0),
      measured_cumulative_(0.0),
      kf_Q_angle_(0.001),
      kf_Q_omega_(0.01),
      kf_R_(0.001) {
  kf_x_ = cv::Mat::zeros(2, 1, CV_64F);
  kf_P_ = cv::Mat::eye(2, 2, CV_64F);
}

void MotionEstimator::Init(const cv::Point2f& center, double radius) {
  (void)center;
  (void)radius;
  initialized_ = true;
  filtered_angle_ = 0.0;
  angular_velocity_ = 0.0;
  measured_cumulative_ = 0.0;
  kf_x_ = cv::Mat::zeros(2, 1, CV_64F);
  kf_P_ = cv::Mat::eye(2, 2, CV_64F);
}

double MotionEstimator::ComputeRotationFromPins(
    const std::vector<Pin>& current_pins,
    const std::vector<Pin>& prev_pins,
    double& confidence) const {
  if (current_pins.size() < 2 || prev_pins.size() < 2) {
    confidence = 0.0;
    return 0.0;
  }

  std::vector<double> angle_diffs;
  for (const auto& cp : current_pins) {
    double best_diff = std::numeric_limits<double>::max();
    for (const auto& pp : prev_pins) {
      double diff = cp.angle_rad - pp.angle_rad;
      while (diff > CV_PI) diff -= 2 * CV_PI;
      while (diff < -CV_PI) diff += 2 * CV_PI;
      if (std::abs(diff) < std::abs(best_diff)) {
        best_diff = diff;
      }
    }
    if (std::abs(best_diff) < 0.5) {
      angle_diffs.push_back(best_diff);
    }
  }

  if (angle_diffs.size() < 2) {
    confidence = 0.0;
    return 0.0;
  }

  std::sort(angle_diffs.begin(), angle_diffs.end());
  size_t mid = angle_diffs.size() / 2;
  double median = angle_diffs[mid];

  std::vector<double> residuals;
  for (auto d : angle_diffs) {
    residuals.push_back(std::abs(d - median));
  }
  std::sort(residuals.begin(), residuals.end());
  double mad = residuals[residuals.size() / 2];
  double std_est = mad * 1.4826;

  std::vector<double> inliers;
  for (auto d : angle_diffs) {
    if (std::abs(d - median) < 2.0 * std_est) {
      inliers.push_back(d);
    }
  }

  if (inliers.size() < 2) {
    confidence = 0.0;
    return 0.0;
  }

  std::sort(inliers.begin(), inliers.end());
  double robust_angle = inliers[inliers.size() / 2];
  confidence = std::min(1.0, inliers.size() / 10.0);

  return robust_angle;
}

double MotionEstimator::EstimateRotation(const std::vector<Pin>& current_pins,
                                         const std::vector<Pin>& prev_pins,
                                         double dt) {
  if (!initialized_) return 0.0;

  double pin_confidence = 0.0;
  double pin_angle = ComputeRotationFromPins(current_pins, prev_pins,
                                              pin_confidence);

  if (pin_confidence < 0.2) {
    return angular_velocity_;
  }

  if (angular_velocity_ != 0 && pin_angle * angular_velocity_ < 0
      && std::abs(pin_angle) > 0.02) {
    kf_x_ = cv::Mat::zeros(2, 1, CV_64F);
    kf_P_ = cv::Mat::eye(2, 2, CV_64F);
    measured_cumulative_ = 0;
    filtered_angle_ = 0;
    angular_velocity_ = 0;
    velocity_confidence_ = 0;
    std::cout << "[Motion] Direction reversal detected, KF reset" << std::endl;
  }

  if (std::abs(angular_velocity_) < 0.1 && std::abs(pin_angle) > 0.01) {
    double q_fast = kf_Q_omega_ * 100;
    double r_fast = kf_R_ * 0.1;
    UpdateState(pin_angle, dt, r_fast, kf_Q_angle_, q_fast);
  } else {
    UpdateState(pin_angle, dt, kf_R_, kf_Q_angle_, kf_Q_omega_);
  }
  return angular_velocity_;
}

void MotionEstimator::UpdateState(double measured_angle, double dt,
                                 double measurement_noise,
                                 double q_angle, double q_omega) {
  if (dt < 1e-6) return;

  measured_cumulative_ += measured_angle;

  cv::Mat F = (cv::Mat_<double>(2, 2) << 1, dt, 0, 1);
  cv::Mat H = (cv::Mat_<double>(1, 2) << 1, 0);

  cv::Mat Q = (cv::Mat_<double>(2, 2) <<
      q_angle * dt * dt * dt / 3.0, q_angle * dt * dt / 2.0,
      q_angle * dt * dt / 2.0, q_omega * dt);
  cv::Mat R = cv::Mat::eye(1, 1, CV_64F) * measurement_noise;

  cv::Mat x_pred = F * kf_x_;
  cv::Mat P_pred = F * kf_P_ * F.t() + Q;

  cv::Mat z = (cv::Mat_<double>(1, 1) << measured_cumulative_);
  cv::Mat y = z - H * x_pred;
  cv::Mat S = H * P_pred * H.t() + R;
  cv::Mat K = P_pred * H.t() * S.inv();

  kf_x_ = x_pred + K * y;
  kf_P_ = (cv::Mat::eye(2, 2, CV_64F) - K * H) * P_pred;

  filtered_angle_ = kf_x_.at<double>(0);
  angular_velocity_ = kf_x_.at<double>(1);
  measured_cumulative_ = filtered_angle_;

  double omega_var = kf_P_.at<double>(1, 1);
  velocity_confidence_ = std::max(0.0, std::min(1.0, 1.0 - omega_var * 10.0));
}

void MotionEstimator::ResetVelocity() {
  angular_velocity_ = 0;
  velocity_confidence_ = 0;
  kf_x_.at<double>(1) = 0;
  kf_P_ = cv::Mat::eye(2, 2, CV_64F);
}

void MotionEstimator::Reset() {
  initialized_ = false;
  filtered_angle_ = 0;
  angular_velocity_ = 0;
  velocity_confidence_ = 0;
  measured_cumulative_ = 0;
  kf_x_ = cv::Mat::zeros(2, 1, CV_64F);
  kf_P_ = cv::Mat::eye(2, 2, CV_64F);
}


