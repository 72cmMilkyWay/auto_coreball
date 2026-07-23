#ifndef AUTO_COREBALL_SCREEN_CAPTURE_HPP
#define AUTO_COREBALL_SCREEN_CAPTURE_HPP

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

class ScreenCapture {
 public:
  ScreenCapture();
  ~ScreenCapture();

  bool FindGameWindow(const std::string& window_name_substring);
  bool OpenDisplay();
  void CloseDisplay();
  bool CaptureFrame(cv::Mat& output);
  Window GetWindow() const { return window_; }
  int GetWindowWidth() const { return width_; }
  int GetWindowHeight() const { return height_; }
  int GetWindowX() const { return x_; }
  int GetWindowY() const { return y_; }

 private:
  Display* display_;
  Window root_;
  Window window_;
  int screen_;
  int width_;
  int height_;
  int x_;
  int y_;
  bool display_owned_;

  std::vector<Window> GetChildWindows(Window parent);
  bool FindWindowRecursive(Window parent, const std::string& substring);
};

#endif
