#include "auto_coreball/visualizer.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>

Visualizer::Visualizer() : is_recording_(false) {}

void Visualizer::Init(const std::string& window_name) {
  window_name_ = window_name;
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
  cv::resizeWindow(window_name_, 960, 720);
}

void Visualizer::Render(const cv::Mat& frame,
                        const GameState& state,
                        const std::vector<GapInfo>& gaps,
                        const ClickDecision& decision,
                        AppState app_state,
                        const std::string& status_msg) {
  if (frame.empty()) return;
  canvas_ = frame.clone();

  DrawPins(canvas_, state.pins);
  const GapInfo* target = decision.should_click ? &decision.target_gap : nullptr;
  DrawGaps(canvas_, gaps, state.center, state.disc_radius, target);
  DrawDecision(canvas_, decision, state.center, state.disc_radius);
  DrawStatus(canvas_, app_state, status_msg);
  DrawGameInfo(canvas_, state);
  DrawRecording(canvas_);
}

void Visualizer::DrawPins(cv::Mat& img, const std::vector<Pin>& pins) {
  for (size_t i = 0; i < pins.size(); ++i) {
    const auto& pin = pins[i];
    cv::Scalar color(0, 255, 255);
    cv::circle(img, pin.outer_pt, 12, color, 2);
    cv::circle(img, pin.outer_pt, 4, cv::Scalar(0, 0, 255), -1);
    cv::line(img, pin.inner_pt, pin.outer_pt, cv::Scalar(255, 0, 0), 1);

    cv::putText(img, std::to_string(i),
                cv::Point(pin.outer_pt.x + 10, pin.outer_pt.y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
  }
}

void Visualizer::DrawGaps(cv::Mat& img, const std::vector<GapInfo>& gaps,
                          const cv::Point2f& center, double radius,
                          const GapInfo* target_gap) {
  if (gaps.empty()) return;

  double outer = radius * 1.0;
  double inner = radius * 0.85;

  for (size_t i = 0; i < gaps.size(); ++i) {
    bool is_target = (target_gap && gaps[i].index == target_gap->index);
    int thickness = is_target ? 6 : 3;
    cv::Scalar color = is_target
        ? cv::Scalar(0, 255, 0)
        : cv::Scalar(100, 100, 100);

    cv::ellipse(img, center, cv::Size(outer, outer), 0,
                gaps[i].start_angle * 180.0 / CV_PI,
                gaps[i].end_angle * 180.0 / CV_PI,
                color, thickness);
    cv::ellipse(img, center, cv::Size(inner, inner), 0,
                gaps[i].start_angle * 180.0 / CV_PI,
                gaps[i].end_angle * 180.0 / CV_PI,
                color, thickness);
  }

  cv::circle(img, center, static_cast<int>(radius),
             cv::Scalar(0, 255, 0), 1);

  double ia = CV_PI / 2;
  cv::Point2f ip(center.x + radius * std::cos(ia),
                 center.y + radius * std::sin(ia));
  cv::line(img, cv::Point(ip.x - 10, ip.y), cv::Point(ip.x + 10, ip.y),
           cv::Scalar(0, 255, 255), 2);
  cv::line(img, cv::Point(ip.x, ip.y - 10), cv::Point(ip.x, ip.y + 10),
           cv::Scalar(0, 255, 255), 2);
}

void Visualizer::DrawDecision(cv::Mat& img, const ClickDecision& decision,
                              const cv::Point2f& center, double radius) {
  if (decision.should_click) {
    // 球命中位置：圆盘底部（插入点）
    cv::Point2f click_pt(
        center.x + radius * std::cos(CV_PI / 2),
        center.y + radius * std::sin(CV_PI / 2));
    cv::circle(img, click_pt, 8, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, click_pt, 12, cv::Scalar(0, 0, 255), 2);

    cv::putText(img, "CLICK!",
                cv::Point(click_pt.x + 15, click_pt.y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 0, 255), 2);
  }
}

void Visualizer::DrawStatus(cv::Mat& img, AppState state,
                            const std::string& msg) {
  cv::Scalar color;
  std::string state_str;

  switch (state) {
    case AppState::INIT:
      color = cv::Scalar(200, 200, 200);
      state_str = "INIT";
      break;
    case AppState::DETECTING_DISC:
      color = cv::Scalar(255, 255, 0);
      state_str = "DETECTING";
      break;
    case AppState::TRACKING:
      color = cv::Scalar(0, 255, 0);
      state_str = "TRACKING";
      break;
    case AppState::ANALYZING_GAP:
      color = cv::Scalar(255, 165, 0);
      state_str = "ANALYZING";
      break;
    case AppState::WAITING:
      color = cv::Scalar(255, 255, 0);
      state_str = "WAITING";
      break;
    case AppState::CLICKING:
      color = cv::Scalar(0, 0, 255);
      state_str = "CLICKING";
      break;
    case AppState::FAILED:
      color = cv::Scalar(0, 0, 255);
      state_str = "FAILED";
      break;
    default:
      color = cv::Scalar(128, 128, 128);
      state_str = "UNKNOWN";
  }

  cv::rectangle(img, cv::Point(5, 5), cv::Point(200, 65),
                cv::Scalar(0, 0, 0), -1);
  cv::putText(img, "State: " + state_str, cv::Point(10, 25),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
  cv::putText(img, msg, cv::Point(10, 50),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
}

void Visualizer::DrawGameInfo(cv::Mat& img, const GameState& state) {
  cv::rectangle(img, cv::Point(5, 70), cv::Point(250, 165),
                cv::Scalar(0, 0, 0), -1);

  std::ostringstream oss;
  oss << "Pins: " << state.pin_count;
  cv::putText(img, oss.str(), cv::Point(10, 90),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

  oss.str("");
  oss << "Omega: " << std::fixed << std::setprecision(3)
      << state.angular_velocity << " rad/s";
  cv::putText(img, oss.str(), cv::Point(10, 108),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

  oss.str("");
  oss << "Vel Conf: " << std::fixed << std::setprecision(2)
      << state.angular_velocity_confidence;
  cv::putText(img, oss.str(), cv::Point(10, 126),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

  oss.str("");
  oss << "Angle: " << std::fixed << std::setprecision(2)
      << state.rotation_angle * 180.0 / CV_PI << " deg";
  cv::putText(img, oss.str(), cv::Point(10, 144),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

  oss.str("");
  oss << "Gaps: " << state.gaps.size();
  cv::putText(img, oss.str(), cv::Point(10, 162),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
}

void Visualizer::DrawRecording(cv::Mat& img) {
  if (is_recording_) {
    cv::circle(img, cv::Point(img.cols - 20, 20), 8,
               cv::Scalar(0, 0, 255), -1);
    cv::putText(img, "REC", cv::Point(img.cols - 45, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
  }
}

void Visualizer::Show() const {
  cv::imshow(window_name_, canvas_);
}

char Visualizer::WaitKey(int delay_ms) const {
  return static_cast<char>(cv::waitKey(delay_ms));
}

bool Visualizer::IsWindowOpen() const {
  return cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) > 0;
}
