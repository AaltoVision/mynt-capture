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

## Remote control via Firebase

1. Install Firebase tools (`sudo`) `pip install --upgrade firebase-admin`
2. Get a Firebase Admin Service JSON key (should not be needed, but easier this way)
   and set the path pointing to it in `src/remote.py`
2. (optional) Disable hibernate on laptop lid close.
   Linux: run `systemd-inhibit --what=handle-lid-switch sleep 1d` and leave
   that terminal window open (Ctrl+C releases the hibernation lock)
3. Run `python src/remote.py`
4. In a special version of `android-viotester`, starting and stopping recording
   will now automatically start & stop `target/mynt-capture`.
