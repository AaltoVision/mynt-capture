# MYNT EYE recording tools

Tested on S1030, on Linux.

## Usage

Requires CMake. To build the project:

* `./scripts/build.sh` to build the dependencies including OpenCV 3 and the MYNT SDK.
* `mkdir target`
* `cd target`
* `cmake ..`
* `make`

To record:

* `cd target`
* `./mynt-capture`
* Check results in `target/output/`.
