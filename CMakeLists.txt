cmake_minimum_required(VERSION 3.0)
project(mynt-capture)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -O2")

include_directories(./)

# Before jsonl-recorder so it doesn't try to use OpenCV 4.
find_package(OpenCV 3 REQUIRED PATHS "build/opencv")
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# Enable OpenCV dependency for jsonl-recorder.
set(USE_OPENCV_VIDEO_RECORDING ON CACHE INTERNAL "")
add_subdirectory("jsonl-recorder")

find_package(mynteye REQUIRED PATHS "MYNT-EYE-S-SDK/_build/install")
message(STATUS "Found mynteye ${mynteye_VERSION}")

add_executable(mynt-capture src/main.cpp)
target_link_libraries(mynt-capture PRIVATE mynteye jsonl-recorder)
