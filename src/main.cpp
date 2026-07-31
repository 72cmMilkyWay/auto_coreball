#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <X11/Xlib.h>
#include <X11/keysym.h>

#include "auto_coreball/screen_capture.hpp"
#include "auto_coreball/image_processor.hpp"
#include "auto_coreball/motion_estimator.hpp"
#include "auto_coreball/game_analyzer.hpp"
#include "auto_coreball/click_controller.hpp"
#include "auto_coreball/visualizer.hpp"

static std::atomic<bool> g_running(true);

void SignalHandler(int) {
  g_running = false;
}

void PrintUsage(const char* prog) {
  std::cout << "Usage: " << prog << " [options]\n"
            << "Options:\n"
            << "  --window <substr>   Window name substring (default: 'coreball')\n"
            << "  --no-viz            Disable visualization\n"
            << "  --debug             Save raw capture frame for debugging\n"
            << "  --help              Show this help\n";
}

int main(int argc, char** argv) {
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::string window_substr = "见缝插针";
  bool enable_viz = true;
  bool debug_save = false;
  bool no_click = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--window" && i + 1 < argc) {
      window_substr = argv[++i];
    } else if (arg == "--no-viz") {
      enable_viz = false;
    } else if (arg == "--debug") {
      debug_save = true;
    } else if (arg == "--no-click") {
      no_click = true;
    } else if (arg == "--help") {
      PrintUsage(argv[0]);
      return 0;
    }
  }

  ScreenCapture capture;
  if (!capture.OpenDisplay()) {
    std::cerr << "Failed to open display" << std::endl;
    return 1;
  }

  int retry = 0;
  while (!capture.FindGameWindow(window_substr) && retry < 30) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    retry++;
  }

  if (!capture.FindGameWindow(window_substr)) {
    std::cerr << "Could not find game window containing '" << window_substr << "'" << std::endl;
    std::cerr << "Try: " << argv[0] << " --window <part_of_window_title>" << std::endl;
    capture.CloseDisplay();
    return 1;
  }

  if (debug_save) {
    cv::Mat debug_frame;
    if (capture.CaptureFrame(debug_frame)) {
      cv::imwrite("debug_raw_frame.png", debug_frame);
    }
  }

  ImageProcessor processor;
  MotionEstimator motion_estimator;
  GameAnalyzer game_analyzer;
  ClickController click_controller;
  Visualizer visualizer;

  click_controller.Initialize(capture.GetWindowX(), capture.GetWindowY());

  if (enable_viz) {
    visualizer.Init("Auto CoreBall");
  }

  GameState game_state{};
  std::vector<Pin> prev_pins;
  AppState app_state = AppState::DETECTING_DISC;
  std::string status_msg = "Looking for disc...";
  int consecutive_failures = 0;

  ClickDecision current_decision{};
  std::chrono::steady_clock::time_point last_click_time = std::chrono::steady_clock::now();

  auto last_frame_time = std::chrono::steady_clock::now();
  bool disc_detected = false;
  cv::Point2f center;
  double radius = 0.0;
  auto disc_detected_time = std::chrono::steady_clock::time_point::min();

  double dynamic_lead_time = 0.08;
  double kMinShotInterval = 0.5;
  const double kInitMeasureTime = 1.0;
  bool awaiting_pin = false;
  int shot_pin_count = 0;
  std::chrono::steady_clock::time_point shot_time;
  int consecutive_misses = 0;
  bool ever_had_pins = false;
  bool game_over = false;
  int zero_pin_frames = 0;
  int burst_shots = 0;
  double last_velocity = 0.0;
  std::chrono::steady_clock::time_point speed_change_time =
      std::chrono::steady_clock::time_point::min();
  std::chrono::steady_clock::time_point reversal_time =
      std::chrono::steady_clock::time_point::min();

  while (g_running) {
    if (click_controller.IsKeyPressed(XK_q)) {
      break;
    }

    cv::Mat frame;
    if (!capture.CaptureFrame(frame)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    cv::Mat gray;
    processor.PreprocessFrame(frame, gray);

    if (!disc_detected) {
      status_msg = "Detecting disc...";
      app_state = AppState::DETECTING_DISC;
      bool found = processor.DetectDisc(gray, center, radius);
      if (found) {
        disc_detected = true;
        motion_estimator.Init(center, radius);
        game_state.center = center;
        game_state.disc_radius = radius;
        status_msg = "Disc detected! Tracking...";
        app_state = AppState::TRACKING;
        game_analyzer.SetInsertionAngle(CV_PI / 2);
        consecutive_failures = 0;
        ever_had_pins = false;
        game_over = false;
        zero_pin_frames = 0;
        last_frame_time = std::chrono::steady_clock::now();
        last_click_time = std::chrono::steady_clock::now();
        disc_detected_time = std::chrono::steady_clock::now();
        burst_shots = 0;
        reversal_time = std::chrono::steady_clock::time_point::min();
        game_analyzer.Reset();
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
    }

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_frame_time).count();
    last_frame_time = now;

    std::vector<Pin> current_pins = processor.DetectPins(gray, center, radius);

    int current_pin_count = static_cast<int>(current_pins.size());

    if (current_pin_count > 0) {
      ever_had_pins = true;
    }

    if (ever_had_pins && current_pin_count == 0) {
      zero_pin_frames++;
      if (zero_pin_frames > 60 && !game_over) {
        game_over = true;
      }
    } else if (current_pin_count > 0) {
      zero_pin_frames = 0;
      if (game_over) {
        game_over = false;
        ever_had_pins = false;
        consecutive_misses = 0;
        disc_detected_time = std::chrono::steady_clock::now();
        burst_shots = 0;
        awaiting_pin = false;
        reversal_time = std::chrono::steady_clock::time_point::min();
      }
    }

    if (current_pin_count < 2) {
      consecutive_failures++;
      if (consecutive_failures > 300) {
        status_msg = "Pin detection failed. Re-detecting disc...";
        app_state = AppState::DETECTING_DISC;
        disc_detected = false;
        motion_estimator.Reset();
        ever_had_pins = false;
        game_over = false;
        zero_pin_frames = 0;
        continue;
      }
    } else {
      consecutive_failures = 0;
    }

    if (awaiting_pin) {
      if (current_pin_count > shot_pin_count) {
        awaiting_pin = false;
        consecutive_misses = 0;
      } else if (std::chrono::duration<double>(
          std::chrono::steady_clock::now() - shot_time).count() > 1.0) {
        awaiting_pin = false;
        consecutive_misses++;
        if (consecutive_misses >= 1) {
          game_over = true;
        }
      }
    }

    double angular_velocity = motion_estimator.EstimateRotation(
        current_pins, prev_pins, dt);

    if (motion_estimator.HasReversed()) {
      reversal_time = std::chrono::steady_clock::now();
    }

    if (std::abs(angular_velocity - last_velocity) > 0.5 &&
        motion_estimator.GetVelocityConfidence() > 0.3) {
      speed_change_time = std::chrono::steady_clock::now();
    }
    last_velocity = angular_velocity;

    game_state.pins = current_pins;
    game_state.pin_count = static_cast<int>(current_pins.size());
    game_state.angular_velocity = angular_velocity;
    game_state.rotation_angle = motion_estimator.GetFilteredAngle();
    game_state.angular_velocity_confidence = motion_estimator.GetVelocityConfidence();

    auto gaps = game_analyzer.FindGaps(current_pins);
    game_state.gaps.clear();
    for (const auto& g : gaps) {
      game_state.gaps.push_back(g.angle_width);
    }

    int unnumbered_count = 0;
    for (const auto& p : current_pins) {
      if (!p.has_number) unnumbered_count++;
    }

    double elapsed_since_detection = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - disc_detected_time).count();
    if (elapsed_since_detection < kInitMeasureTime) {
      status_msg = "Initial measurement...";
      app_state = AppState::TRACKING;
    } else if (game_over) {
      status_msg = "Game over. Waiting for restart...";
      app_state = AppState::WAITING;
    } else if (burst_shots < 15 && unnumbered_count < 2 && !no_click) {
      auto since_last = std::chrono::steady_clock::now() - last_click_time;
      if (since_last > std::chrono::milliseconds(100)) {
        double click_x = center.x;
        double click_y = center.y + radius * 4;
        click_controller.ClickOnWindow(static_cast<int>(click_x),
                                       static_cast<int>(click_y));
        last_click_time = std::chrono::steady_clock::now();
        awaiting_pin = true;
        shot_pin_count = current_pin_count;
        shot_time = last_click_time;
        ++burst_shots;
        status_msg = "Burst " + std::to_string(burst_shots) + "/15";
      } else {
        status_msg = "Burst: waiting...";
      }
    } else if (motion_estimator.GetVelocityConfidence() > 0.15) {
      auto warmup_elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - disc_detected_time).count();
      if (warmup_elapsed < 1.0) {
        status_msg = "Warming up...";
      } else {
        if (awaiting_pin) {
          status_msg = "Waiting for pin...";
        } else {
          double since_last_shot = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - last_click_time).count();
          if (speed_change_time != std::chrono::steady_clock::time_point::min()) {
            double sc_elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - speed_change_time).count();
            if (sc_elapsed < 0.5) {
              status_msg = "Speed change: measuring...";
              goto skip_shot;
            }
          }
          if (reversal_time != std::chrono::steady_clock::time_point::min()) {
            double rev_elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - reversal_time).count();
            if (rev_elapsed < 0.3) {
              status_msg = "Reversal: measuring...";
              goto skip_shot;
            } else if (rev_elapsed < 3.0) {
              kMinShotInterval = 0.5;
            } else {
              kMinShotInterval = 10.0;
            }
          } else {
            kMinShotInterval = 0.5;
          }
          if (since_last_shot < kMinShotInterval) {
            status_msg = "Shot cooldown...";
            goto skip_shot;
          }
          app_state = AppState::ANALYZING_GAP;
          current_decision = game_analyzer.DecideClick(
              gaps, game_analyzer.GetInsertionAngle(),
              angular_velocity,
              motion_estimator.GetVelocityConfidence(),
              dynamic_lead_time);

          if (current_decision.should_click && current_decision.confidence > 0.2
              && processor.IsRadiusLearned() && !no_click) {
            app_state = AppState::CLICKING;
            double click_x = center.x;
            double click_y = center.y + radius * 4;
            click_controller.ClickOnWindow(static_cast<int>(click_x),
                                           static_cast<int>(click_y));
            last_click_time = std::chrono::steady_clock::now();
            awaiting_pin = true;
            shot_pin_count = current_pin_count;
            shot_time = last_click_time;
            status_msg = "CLICK!";
          } else {
            status_msg = "Analyzing gaps...";
          }
skip_shot: ;
        }
      }
    } else {
      status_msg = "Estimating velocity...";
      app_state = AppState::TRACKING;
    }

    if (enable_viz) {
      std::vector<GapInfo> all_gaps = gaps;
      visualizer.Render(frame, game_state, all_gaps, current_decision,
                        app_state, status_msg);
      visualizer.Show();

      char key = visualizer.WaitKey(1);
      if (key == 'q' || key == 27) {
        break;
      }
    }

    prev_pins = current_pins;
  }

  cv::destroyAllWindows();
  capture.CloseDisplay();
  return 0;
}
