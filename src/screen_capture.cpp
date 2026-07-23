#include "auto_coreball/screen_capture.hpp"
#include <opencv2/imgproc.hpp>
#include <cstring>
#include <iostream>

ScreenCapture::ScreenCapture()
    : display_(nullptr), root_(0), window_(0), screen_(0),
      width_(0), height_(0), x_(0), y_(0), display_owned_(false) {}

ScreenCapture::~ScreenCapture() {
  CloseDisplay();
}

bool ScreenCapture::OpenDisplay() {
  display_ = XOpenDisplay(nullptr);
  if (!display_) {
    std::cerr << "[ScreenCapture] Cannot open X display" << std::endl;
    return false;
  }
  screen_ = DefaultScreen(display_);
  root_ = RootWindow(display_, screen_);
  display_owned_ = true;
  return true;
}

void ScreenCapture::CloseDisplay() {
  if (display_ && display_owned_) {
    XCloseDisplay(display_);
    display_ = nullptr;
    display_owned_ = false;
  }
}

std::vector<Window> ScreenCapture::GetChildWindows(Window parent) {
  std::vector<Window> result;
  Window root_ret, parent_ret;
  Window* children;
  unsigned int nchildren;
  if (XQueryTree(display_, parent, &root_ret, &parent_ret,
                 &children, &nchildren)) {
    for (unsigned int i = 0; i < nchildren; ++i) {
      result.push_back(children[i]);
    }
    if (children) XFree(children);
  }
  return result;
}

bool ScreenCapture::FindWindowRecursive(Window parent,
                                        const std::string& substring) {
  std::string window_name;

  Atom net_wm_name = XInternAtom(display_, "_NET_WM_NAME", True);

  if (net_wm_name != None) {
    XTextProperty tp;
    if (XGetTextProperty(display_, parent, &tp, net_wm_name) && tp.value) {
      window_name = reinterpret_cast<char*>(tp.value);
      XFree(tp.value);
    }
  }

  if (window_name.empty()) {
    char* name = nullptr;
    if (XFetchName(display_, parent, &name) && name) {
      window_name = name;
      XFree(name);
    }
  }

  if (!window_name.empty() &&
      window_name.find(substring) != std::string::npos) {
    window_ = parent;
    XWindowAttributes attr;
    XGetWindowAttributes(display_, window_, &attr);
    width_ = attr.width;
    height_ = attr.height;

    Window child;
    XTranslateCoordinates(display_, window_, root_,
                          0, 0, &x_, &y_, &child);
    return true;
  }
  auto children = GetChildWindows(parent);
  for (auto child : children) {
    if (FindWindowRecursive(child, substring)) return true;
  }
  return false;
}

bool ScreenCapture::FindGameWindow(const std::string& window_name_substring) {
  if (!display_) {
    if (!OpenDisplay()) return false;
  }
  return FindWindowRecursive(root_, window_name_substring);
}

bool ScreenCapture::CaptureFrame(cv::Mat& output) {
  if (!display_ || !window_) return false;

  XWindowAttributes attr;
  if (!XGetWindowAttributes(display_, window_, &attr)) return false;

  width_ = attr.width;
  height_ = attr.height;

  XImage* img = XGetImage(display_, window_, 0, 0,
                          width_, height_,
                          AllPlanes, ZPixmap);
  if (!img) return false;

  cv::Mat raw(height_, width_, CV_8UC4, img->data);
  cv::cvtColor(raw, output, cv::COLOR_BGRA2BGR);
  XDestroyImage(img);
  return true;
}
