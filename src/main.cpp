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

  std::cout << "=== Auto CoreBall (见缝插针) ===" << std::endl;
  std::cout << "Visual SLAM based auto-player" << std::endl;
  std::cout << std::endl;
  std::cout << "Looking for window containing: '" << window_substr << "'" << std::endl;
  std::cout << "Make sure the game is open in your browser." << std::endl;
  std::cout << "Press Ctrl+C to exit." << std::endl;
  std::cout << std::endl;

  ScreenCapture capture;
  if (!capture.OpenDisplay()) {
    std::cerr << "Failed to open display" << std::endl;
    return 1;
  }

  std::cout << "Searching for game window..." << std::endl;
  int retry = 0;
  while (!capture.FindGameWindow(window_substr) && retry < 30) {
    std::cout << "  Retry " << (retry + 1) << "/30..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    retry++;
  }

  if (!capture.FindGameWindow(window_substr)) {
    std::cerr << "Could not find game window containing '" << window_substr << "'" << std::endl;
    std::cerr << "Try: " << argv[0] << " --window <part_of_window_title>" << std::endl;
    capture.CloseDisplay();
    return 1;
  }

  int win_w = capture.GetWindowWidth();
  int win_h = capture.GetWindowHeight();
  std::cout << "Found game window: "
            << win_w << "x" << win_h
            << " at (" << capture.GetWindowX() << ", " << capture.GetWindowY() << ")"
            << std::endl;

  if (debug_save) {
    cv::Mat debug_frame;
    if (capture.CaptureFrame(debug_frame)) {
      cv::imwrite("debug_raw_frame.png", debug_frame);
      std::cout << "Saved debug_raw_frame.png - check this to see what the program sees"
                << std::endl;
    }
  }

  ImageProcessor processor;
  MotionEstimator motion_estimator;
  GameAnalyzer game_analyzer;
  ClickController click_controller;
  Visualizer visualizer;

  click_controller.Initialize(capture.GetWindowX(), capture.GetWindowY());

  if (enable_viz) {
    visualizer.Init("Auto CoreBall - Visual SLAM");
  }

  GameState game_state{};
  std::vector<Pin> prev_pins;
  AppState app_state = AppState::DETECTING_DISC;
  std::string status_msg = "Looking for disc...";
  int consecutive_failures = 0;

  ClickDecision current_decision{};
  std::chrono::steady_clock::time_point last_click_time = std::chrono::steady_clock::now();

  int frame_count = 0;
  auto last_frame_time = std::chrono::steady_clock::now();
  bool disc_detected = false;
  cv::Point2f center;
  double radius = 0.0;
  auto disc_detected_time = std::chrono::steady_clock::time_point::min();

  double dynamic_lead_time = 0.11;
  const double kMinShotInterval = 0.5;
  const double kInitMeasureTime = 1.0;
  bool awaiting_pin = false;
  int shot_pin_count = 0;
  std::chrono::steady_clock::time_point shot_time;
  int consecutive_misses = 0;
  bool ever_had_pins = false;
  bool game_over = false;
  int zero_pin_frames = 0;

  while (g_running) {
    if (click_controller.IsKeyPressed(XK_q)) {
      std::cout << "\nQ pressed, exiting..." << std::endl;
      break;
    }

    auto loop_start = std::chrono::steady_clock::now();
    frame_count++;

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
      if (frame_count % 30 == 0) {
        std::cout << "[Frame " << frame_count << "] DetectDisc: "
                  << (found ? "FOUND" : "not found")
                  << " (window=" << capture.GetWindowWidth()
                  << "x" << capture.GetWindowHeight() << ")"
                  << std::endl;
      }
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

    if (frame_count % 30 == 0) {
      std::cout << "[Frame " << frame_count << "] Disc: ("
                << static_cast<int>(center.x) << ","
                << static_cast<int>(center.y) << ") r="
                << static_cast<int>(radius)
                << " Pins: " << current_pins.size();
      if (!current_pins.empty()) {
        std::cout << " Angles:";
        for (const auto& p : current_pins) {
          std::cout << " " << (p.angle_rad * 180.0 / CV_PI);
        }
      }
      std::cout << std::endl;
    }

    int current_pin_count = static_cast<int>(current_pins.size());

    if (current_pin_count > 0) {
      ever_had_pins = true;
    }

    if (ever_had_pins && current_pin_count == 0) {
      zero_pin_frames++;
      if (zero_pin_frames > 60 && !game_over) {
        game_over = true;
        std::cout << "[Game] Game over detected, stopping clicks" << std::endl;
      }
    } else if (current_pin_count > 0) {
      zero_pin_frames = 0;
      if (game_over) {
        game_over = false;
        ever_had_pins = false;
        consecutive_misses = 0;
        disc_detected_time = std::chrono::steady_clock::now();
        awaiting_pin = false;
        motion_estimator.ResetVelocity();
        std::cout << "[Game] Game restarted, resuming" << std::endl;
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

    // 延迟测量
    if (awaiting_pin) {
      if (current_pin_count > shot_pin_count) {
        double delay = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - shot_time).count();
        awaiting_pin = false;
        consecutive_misses = 0;
        std::cout << "[Timing] Actual delay = " << (delay * 1000.0)
                  << " ms (click to new pin)" << std::endl;
      } else if (std::chrono::duration<double>(
          std::chrono::steady_clock::now() - shot_time).count() > 1.0) {
        awaiting_pin = false;
        consecutive_misses++;
        std::cout << "[Timing] Miss #" << consecutive_misses << std::endl;
        if (consecutive_misses >= 1) {
          game_over = true;
          std::cout << "[Game] Game over detected (shot missed)" << std::endl;
        }
      }
    }

    double angular_velocity = motion_estimator.EstimateRotation(
        current_pins, prev_pins, dt);

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

    if (frame_count % 30 == 0 && !gaps.empty()) {
      auto after_analysis = std::chrono::steady_clock::now();
      double process_ms = std::chrono::duration<double>(after_analysis - loop_start).count() * 1000.0;

      std::string dir = (angular_velocity > 0) ? "CW" : "CCW";
      std::cout << "  Gaps: " << gaps.size()
                << " Best: " << (gaps[0].angle_width * 180.0 / CV_PI) << " deg"
                << " Vel: " << angular_velocity << " rad/s (" << dir << ")"
                << " Conf: " << motion_estimator.GetVelocityConfidence()
                << " Lead: " << dynamic_lead_time
                << " Proc: " << static_cast<int>(process_ms) << "ms"
                << std::endl;

      if (current_pins.size() >= 2 && prev_pins.size() >= 2) {
        std::cout << "  Pin movement (cur - prev):";
        for (size_t i = 0; i < current_pins.size() && i < 3; ++i) {
          double diff = current_pins[i].angle_rad - prev_pins[i].angle_rad;
          while (diff > CV_PI) diff -= 2 * CV_PI;
          while (diff < -CV_PI) diff += 2 * CV_PI;
          std::cout << " " << (diff * 180.0 / CV_PI) << "deg";
        }
        std::cout << " (+ = angle increase)" << std::endl;
      }
    }

    // =============================================
    // 2. 决策与击发
    // =============================================
    double elapsed_since_detection = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - disc_detected_time).count();
    if (elapsed_since_detection < kInitMeasureTime) {
      status_msg = "Initial measurement...";
      app_state = AppState::TRACKING;
    } else if (game_over) {
      status_msg = "Game over. Waiting for restart...";
      app_state = AppState::WAITING;
    } else if (current_pins.size() < 2 && !no_click) {
      app_state = AppState::TRACKING;
      auto since_last = std::chrono::steady_clock::now() - last_click_time;
      bool first_fire = (last_click_time - disc_detected_time) < std::chrono::milliseconds(50);
      auto delay = first_fire ? std::chrono::milliseconds(1000) : std::chrono::milliseconds(500);
      if (since_last > delay) {
        double click_x = center.x;
        double click_y = center.y + radius * 4;
        click_controller.ClickOnWindow(static_cast<int>(click_x),
                                       static_cast<int>(click_y));
        last_click_time = std::chrono::steady_clock::now();
        awaiting_pin = true;
        shot_pin_count = current_pin_count;
        shot_time = last_click_time;
        status_msg = "Startup CLICK!";
        std::cout << "[Startup] Blind fire, pins=" << current_pins.size()
                  << " vel=" << angular_velocity << " rad/s" << std::endl;
      } else {
        status_msg = "Startup: waiting...";
      }
    } else if (motion_estimator.GetVelocityConfidence() > 0.5) {
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
          if (since_last_shot < kMinShotInterval) {
            status_msg = "Shot cooldown...";
            goto skip_shot;
          }
          app_state = AppState::ANALYZING_GAP;
          current_decision = game_analyzer.DecideClick(
              gaps, game_analyzer.GetInsertionAngle(),
              angular_velocity, motion_estimator.GetFilteredAngle(),
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
            std::cout << "CLICK! Fired with lead time: " << dynamic_lead_time
                      << "s, gap=" << (current_decision.target_gap.angle_width * 180.0 / CV_PI)
                      << "deg, vel=" << angular_velocity << " rad/s"
                      << std::endl;
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

  std::cout << "\nExiting..." << std::endl;
  cv::destroyAllWindows();
  capture.CloseDisplay();
  return 0;
}
