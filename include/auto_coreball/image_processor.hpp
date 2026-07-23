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
  cv::Point2f FindInsertionPoint(const cv::Mat& gray,
                                 const cv::Point2f& center,
                                 double radius,
                                 bool& found);
  void PreprocessFrame(const cv::Mat& input, cv::Mat& output);
  cv::Rect GetCenterROI(int width, int height) const;

  void SetDebugDraw(bool enabled) { debug_draw_ = enabled; }
  cv::Mat GetDebugImage() const { return debug_img_.clone(); }
  void SaveDebugImage(const std::string& path) const;
  bool IsRadiusLearned() const { return radius_learned_; }

 private:
  bool debug_draw_;
  cv::Mat debug_img_;
  double roi_ratio_;

  std::deque<std::vector<NumberedBall>> ball_history_;
  std::vector<double> rotation_samples_;
  double rotation_radius_;
  int frames_since_detection_;
  bool radius_learned_;

  bool IsWaitingBall(const cv::Point2f& center, double disc_radius) const;

  double ComputeAngle(const cv::Point2f& pt, const cv::Point2f& center);
  bool IsLineThroughCircle(const cv::Vec4i& line, const cv::Point2f& center,
                           double radius, double tolerance);
  double ComputeLineAngle(const cv::Vec4i& line, const cv::Point2f& center);
};

#endif
