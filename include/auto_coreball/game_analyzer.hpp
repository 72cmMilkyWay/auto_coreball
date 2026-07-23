#ifndef AUTO_COREBALL_GAME_ANALYZER_HPP
#define AUTO_COREBALL_GAME_ANALYZER_HPP

#include <vector>
#include <opencv2/core.hpp>
#include "types.hpp"

struct GapInfo {
  double angle_center;
  double angle_width;
  double start_angle;
  double end_angle;
  int index;
};

struct ClickDecision {
  bool should_click;
  double wait_time_ms;
  double click_angle;
  GapInfo target_gap;
  double confidence;
};

class GameAnalyzer {
 public:
  GameAnalyzer();

  void SetInsertionAngle(double angle) { insertion_angle_ = angle; }
  double GetInsertionAngle() const { return insertion_angle_; }

  std::vector<GapInfo> FindGaps(const std::vector<Pin>& pins);
  ClickDecision DecideClick(const std::vector<GapInfo>& gaps,
                            double insertion_angle,
                            double angular_velocity,
                            double current_angle,
                            double velocity_confidence,
                            double dynamic_lead_time);

  void Reset();

  void SetBallFlightTime(double t) { ball_flight_time_ = t; }
  void SetBallDelay(double t) { ball_delay_ = t; }

 private:
  double insertion_angle_;
  double min_gap_width_;
  double ball_delay_;
  double ball_flight_time_;
};

#endif
