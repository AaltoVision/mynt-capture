# MYNT EYE S SDK recording tools

Tested on MYNT EYE S1030, on Linux. Seems to compile on Mac OS but will not run.

## Usage

Requires CMake. To build the project:

* `git submodule update --init --recursive`
* (On Mac) `brew install libuvc`
* `./scripts/build.sh` to build the dependencies including OpenCV 3 and the MYNT SDK.
* `mkdir target`
* `cd target`
* `cmake ..`
* `make`

To record:

* Plug in the device via high-speed USB port.
* `cd target`
* `./mynt-capture`
* Check results in `target/output/`.
