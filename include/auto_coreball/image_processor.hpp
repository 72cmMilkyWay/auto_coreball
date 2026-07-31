#ifndef AUTO_COREBALL_IMAGE_PROCESSOR_HPP
#define AUTO_COREBALL_IMAGE_PROCESSOR_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <deque>
#include "types.hpp"

struct NumberedBall {
  cv::Point2f center;
  double radius;
  double angle_rad;
  int number;
  bool has_number;
};

class ImageProcessor {
 public:
  ImageProcessor();

  bool DetectDisc(const cv::Mat& gray, cv::Point2f& center, double& radius);
  std::vector<Pin> DetectPins(const cv::Mat& gray, const cv::Point2f& center,
                              double radius);
  std::vector<NumberedBall> DetectNumberedBalls(const cv::Mat& gray,
                                                const cv::Point2f& center,
                                                double disc_radius);
  void PreprocessFrame(const cv::Mat& input, cv::Mat& output);

  bool IsRadiusLearned() const { return radius_learned_; }

 private:
  std::deque<std::vector<NumberedBall>> ball_history_;
  std::vector<double> rotation_samples_;
  double rotation_radius_;
  bool radius_learned_;

  double ComputeAngle(const cv::Point2f& pt, const cv::Point2f& center);
};

#endif
