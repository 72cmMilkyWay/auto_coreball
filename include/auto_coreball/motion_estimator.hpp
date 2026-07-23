#ifndef AUTO_COREBALL_MOTION_ESTIMATOR_HPP
#define AUTO_COREBALL_MOTION_ESTIMATOR_HPP

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include "types.hpp"

class MotionEstimator {
 public:
  MotionEstimator();

  void Init(const cv::Point2f& center, double radius);
  double EstimateRotation(const std::vector<Pin>& current_pins,
                           const std::vector<Pin>& prev_pins,
                           double dt);
  void UpdateState(double measured_angle, double dt, double measurement_noise,
                   double q_angle, double q_omega);

  void Reset();

  double GetFilteredAngle() const { return filtered_angle_; }
  double GetAngularVelocity() const { return angular_velocity_; }
  double GetVelocityConfidence() const { return velocity_confidence_; }
  bool IsInitialized() const { return initialized_; }

 private:
  bool initialized_;
  double filtered_angle_;
  double angular_velocity_;
  double velocity_confidence_;
  double measured_cumulative_;

  cv::Mat kf_x_;
  cv::Mat kf_P_;
  double kf_Q_angle_;
  double kf_Q_omega_;
  double kf_R_;

  double ComputeRotationFromPins(const std::vector<Pin>& current_pins,
                                  const std::vector<Pin>& prev_pins,
                                  double& confidence) const;
};

#endif
