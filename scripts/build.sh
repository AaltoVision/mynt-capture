#!/bin/bash

set -e

ROOT_DIR=`pwd`

# Mynt requires OpenCV 3, version 4 won't work.
# The docs use 3.4.1, which by default fails to build for me so below we add some -D switches
# to disable the problematic code.
# Version 3.4.12 and some others won't work because of this: https://github.com/slightech/MYNT-EYE-S-SDK/issues/92
mkdir -p build/opencv
cd build/opencv
cmake -DWITH_JASPER=OFF -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF "$ROOT_DIR/opencv"
make -j4

# Install into a custom directory without using `sudo`.
# `make samples` does not seem to work with a custom install directory, so
# you need to remove `SUDO=` and `-DCMAKE_INSTALL_PREFIX` to build the examples.
cd "$ROOT_DIR/MYNT-EYE-S-SDK"
OpenCV_DIR="$ROOT_DIR/build/opencv" make install \
  SUDO= \
  CMAKE_BUILD_EXTRA_OPTIONS="-DCMAKE_INSTALL_PREFIX=install"
