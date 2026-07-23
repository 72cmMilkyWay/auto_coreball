#include "auto_coreball/game_analyzer.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

GameAnalyzer::GameAnalyzer()
    : insertion_angle_(CV_PI / 2),
      min_gap_width_(0.06),
      ball_delay_(0.0),
      ball_flight_time_(0.0) {}

std::vector<GapInfo> GameAnalyzer::FindGaps(const std::vector<Pin>& pins) {
  std::vector<GapInfo> gaps;
  if (pins.size() < 2) {
    GapInfo full;
    full.angle_center = insertion_angle_;
    full.angle_width = 2 * CV_PI;
    full.start_angle = -CV_PI;
    full.end_angle = CV_PI;
    full.index = 0;
    gaps.push_back(full);
    return gaps;
  }

  for (size_t i = 0; i < pins.size() - 1; ++i) {
    double width = pins[i + 1].angle_rad - pins[i].angle_rad;
    GapInfo gap;
    gap.start_angle = pins[i].angle_rad;
    gap.end_angle = pins[i + 1].angle_rad;
    gap.angle_width = width;
    gap.angle_center = (pins[i].angle_rad + pins[i + 1].angle_rad) / 2.0;
    gap.index = static_cast<int>(i);
    gaps.push_back(gap);
  }

  double wrap_width = (pins[0].angle_rad + 2 * CV_PI) - pins.back().angle_rad;
  GapInfo wrap_gap;
  wrap_gap.start_angle = pins.back().angle_rad;
  wrap_gap.end_angle = pins[0].angle_rad + 2 * CV_PI;
  wrap_gap.angle_width = wrap_width;
  double wrap_center = pins.back().angle_rad + wrap_width / 2.0;
  while (wrap_center > CV_PI) wrap_center -= 2 * CV_PI;
  wrap_gap.angle_center = wrap_center;
  wrap_gap.index = static_cast<int>(pins.size() - 1);
  gaps.push_back(wrap_gap);

  auto cmp = [](const GapInfo& a, const GapInfo& b) {
    return a.angle_width > b.angle_width;
  };
  std::sort(gaps.begin(), gaps.end(), cmp);

  return gaps;
}

ClickDecision GameAnalyzer::DecideClick(const std::vector<GapInfo>& gaps,
                                        double insertion_angle,
                                        double angular_velocity,
                                        double current_angle,
                                        double velocity_confidence,
                                        double dynamic_lead_time) {
  ClickDecision decision;
  decision.should_click = false;
  decision.wait_time_ms = 0.0;
  decision.confidence = 0.0;

  if (gaps.empty() || std::abs(angular_velocity) < 0.01) {
    return decision;
  }

  double rotation_period = 2 * CV_PI / std::abs(angular_velocity);
  double gap_spacing = 2 * CV_PI / static_cast<double>(gaps.size());
  double window_angle = std::min(gap_spacing * 0.25, CV_PI / 6.0);
  double window = window_angle / std::abs(angular_velocity);

  double target_upper = dynamic_lead_time + 0.015;
  double target_lower = target_upper - window;

  const GapInfo* best_gap = nullptr;
  double best_width = 0.0;

  for (const auto& gap : gaps) {
    if (gap.angle_width < min_gap_width_ * 1.2) continue;

    double angular_dist = gap.angle_center - insertion_angle;
    while (angular_dist > CV_PI) angular_dist -= 2 * CV_PI;
    while (angular_dist < -CV_PI) angular_dist += 2 * CV_PI;

    double time_to_reach = -angular_dist / angular_velocity;
    while (time_to_reach < 0) time_to_reach += rotation_period;

    if (time_to_reach <= target_upper && time_to_reach > target_lower) {
      if (gap.angle_width > best_width) {
        best_width = gap.angle_width;
        best_gap = &gap;
      }
    }
  }

  if (best_gap && static_cast<int>(gaps.size()) <= 15
      && best_width < gaps[0].angle_width * 0.9) {
    best_gap = nullptr;
  }

  if (best_gap) {
    decision.should_click = true;
    decision.target_gap = *best_gap;
    decision.click_angle = best_gap->angle_center;
    decision.confidence = velocity_confidence;
  }

  return decision;
}

void GameAnalyzer::Reset() {
  insertion_angle_ = CV_PI / 2;
  ball_flight_time_ = 0.3;
  ball_delay_ = 0.05;
}
