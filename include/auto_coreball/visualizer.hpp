#ifndef AUTO_COREBALL_VISUALIZER_HPP
#define AUTO_COREBALL_VISUALIZER_HPP

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include "types.hpp"
#include "game_analyzer.hpp"

class Visualizer {
 public:
  Visualizer();

  void Init(const std::string& window_name);
  void Render(const cv::Mat& frame,
              const GameState& state,
              const std::vector<GapInfo>& gaps,
              const ClickDecision& decision,
              AppState app_state,
              const std::string& status_msg);

  void Show() const;
  char WaitKey(int delay_ms) const;
  bool IsWindowOpen() const;

  void SetRecording(bool recording) { is_recording_ = recording; }

 private:
  std::string window_name_;
  cv::Mat canvas_;
  bool is_recording_;

  void DrawGameInfo(cv::Mat& img, const GameState& state);
  void DrawGaps(cv::Mat& img, const std::vector<GapInfo>& gaps,
                const cv::Point2f& center, double radius,
                const GapInfo* target_gap = nullptr);
  void DrawDecision(cv::Mat& img, const ClickDecision& decision,
                    const cv::Point2f& center, double radius);
  void DrawStatus(cv::Mat& img, AppState state, const std::string& msg);
  void DrawPins(cv::Mat& img, const std::vector<Pin>& pins,
                const cv::Point2f& center);
  void DrawRecording(cv::Mat& img);
};

#endif
