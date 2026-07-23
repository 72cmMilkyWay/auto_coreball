#ifndef AUTO_COREBALL_TYPES_HPP
#define AUTO_COREBALL_TYPES_HPP

#include <string>
#include <vector>
#include <chrono>

struct Pin {
  int id;
  double angle_rad;
  cv::Point2f inner_pt;
  cv::Point2f outer_pt;
  double length;
};

struct GameState {
  cv::Point2f center;
  double disc_radius;
  double rotation_angle;
  double angular_velocity;
  double angular_velocity_confidence;
  std::vector<Pin> pins;
  std::vector<double> gaps;
  int pin_count;
  int score;
  bool is_alive;
};

enum class AppState {
  INIT,
  DETECTING_DISC,
  TRACKING,
  ANALYZING_GAP,
  WAITING,
  CLICKING,
  PAUSED,
  FAILED
};

struct FrameData {
  cv::Mat color;
  cv::Mat gray;
  std::chrono::steady_clock::time_point timestamp;
};

struct ClickCommand {
  int x;
  int y;
  bool execute;
};

#endif
