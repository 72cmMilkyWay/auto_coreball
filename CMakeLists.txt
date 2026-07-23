cmake_minimum_required(VERSION 3.10)
project(auto_coreball VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4.4 REQUIRED COMPONENTS core imgproc highgui features2d video)
find_package(X11 REQUIRED)

add_executable(auto_coreball
  src/main.cpp
  src/screen_capture.cpp
  src/image_processor.cpp
  src/motion_estimator.cpp
  src/game_analyzer.cpp
  src/click_controller.cpp
  src/visualizer.cpp
)

target_include_directories(auto_coreball PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${X11_INCLUDE_DIR}
)

target_link_libraries(auto_coreball PRIVATE
  ${OpenCV_LIBS}
  ${X11_LIBRARIES}
  ${X11_Xtst_LIB}
  pthread
)

target_compile_options(auto_coreball PRIVATE -Wall -Wextra -O2)
