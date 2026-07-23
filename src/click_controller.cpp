#include "auto_coreball/click_controller.hpp"
#include <X11/extensions/XTest.h>
#include <X11/Xatom.h>
#include <iostream>

ClickController::ClickController()
    : display_(nullptr), window_x_(0), window_y_(0), initialized_(false) {}

ClickController::~ClickController() {
  if (display_) {
    XCloseDisplay(display_);
  }
}

bool ClickController::Initialize(int window_x, int window_y) {
  display_ = XOpenDisplay(nullptr);
  if (!display_) {
    std::cerr << "[ClickController] Cannot open X display" << std::endl;
    return false;
  }
  window_x_ = window_x;
  window_y_ = window_y;
  initialized_ = true;
  return true;
}

void ClickController::ClickAt(int screen_x, int screen_y) {
  if (!initialized_ || !display_) return;

  XTestFakeMotionEvent(display_, -1, screen_x, screen_y, CurrentTime);
  XTestFakeButtonEvent(display_, 1, True, CurrentTime);
  XTestFakeButtonEvent(display_, 1, False, CurrentTime);
  XFlush(display_);
}

void ClickController::ClickOnWindow(int window_relative_x,
                                    int window_relative_y) {
  ClickAt(window_x_ + window_relative_x, window_y_ + window_relative_y);
}

void ClickController::MoveTo(int screen_x, int screen_y) {
  if (!initialized_ || !display_) return;

  XTestFakeMotionEvent(display_, -1, screen_x, screen_y, CurrentTime);
  XFlush(display_);
}

bool ClickController::IsGameWindowActive(Window game_window) const {
  if (!display_) return false;

  Atom active_atom = XInternAtom(display_, "_NET_ACTIVE_WINDOW", True);
  if (active_atom == None) return false;

  Atom actual_type;
  int actual_format;
  unsigned long nitems, bytes_after;
  unsigned char* data = nullptr;

  Status status = XGetWindowProperty(display_, DefaultRootWindow(display_),
                                      active_atom, 0, 1, False,
                                      XA_WINDOW, &actual_type, &actual_format,
                                      &nitems, &bytes_after, &data);
  if (status != Success || !data) return false;

  Window active_window = *reinterpret_cast<Window*>(data);
  XFree(data);

  return active_window == game_window;
}

bool ClickController::IsKeyPressed(KeySym keysym) const {
  if (!display_) return false;
  KeyCode kc = XKeysymToKeycode(display_, keysym);
  if (kc == 0) return false;
  char keys[32];
  XQueryKeymap(display_, keys);
  return (keys[kc / 8] & (1 << (kc % 8))) != 0;
}
