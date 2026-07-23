#ifndef AUTO_COREBALL_CLICK_CONTROLLER_HPP
#define AUTO_COREBALL_CLICK_CONTROLLER_HPP

#include <X11/Xlib.h>

class ClickController {
 public:
  ClickController();
  ~ClickController();

  bool Initialize(int window_x, int window_y);
  void ClickAt(int screen_x, int screen_y);
  void ClickOnWindow(int window_relative_x, int window_relative_y);
  void MoveTo(int screen_x, int screen_y);
  bool IsGameWindowActive(Window game_window) const;

  bool IsKeyPressed(KeySym keysym) const;

  void SetWindowOffset(int wx, int wy) {
    window_x_ = wx;
    window_y_ = wy;
  }

 private:
  Display* display_;
  int window_x_;
  int window_y_;
  bool initialized_;
};

#endif
