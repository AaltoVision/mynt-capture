#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/api/api.h"
#include "mynteye/logger.h"
#include "mynteye/device/device.h"

#include <jsonl-recorder/recorder.hpp>

#include <iostream>
#include <iomanip>
#include <chrono>

std::string currentISO8601TimeUTC() {
  auto now = std::chrono::system_clock::now();
  auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(gmtime(&itt), "%FT%H-%M-%SZ");

  return ss.str();
}

std::shared_ptr<mynteye::API> setupMynt(int argc, char *argv[]) {
  std::shared_ptr<mynteye::API> api = mynteye::API::Create(argc, argv);
  assert(api);

  std::cout << std::endl;
  LOG(INFO) << "Device name: " << api->GetInfo(mynteye::Info::DEVICE_NAME);
  LOG(INFO) << "Serial number: " << api->GetInfo(mynteye::Info::SERIAL_NUMBER);
  LOG(INFO) << "Firmware version: " << api->GetInfo(mynteye::Info::FIRMWARE_VERSION);
  LOG(INFO) << "Hardware version: " << api->GetInfo(mynteye::Info::HARDWARE_VERSION);
  LOG(INFO) << "Spec version: " << api->GetInfo(mynteye::Info::SPEC_VERSION);
  LOG(INFO) << "Lens type: " << api->GetInfo(mynteye::Info::LENS_TYPE);
  LOG(INFO) << "IMU type: " << api->GetInfo(mynteye::Info::IMU_TYPE);
  LOG(INFO) << "Nominal baseline: " << api->GetInfo(mynteye::Info::NOMINAL_BASELINE);

  // std::cout << std::endl;
  // LOG(INFO) << "Intrinsics left: {" << *api->GetIntrinsicsBase(mynteye::Stream::LEFT) << "}";
  // LOG(INFO) << "Intrinsics right: {" << *api->GetIntrinsicsBase(mynteye::Stream::RIGHT) << "}";
  // LOG(INFO) << "Extrinsics right to left: {" << api->GetExtrinsics(mynteye::Stream::RIGHT, mynteye::Stream::LEFT) << "}";

  // std::cout << std::endl;
  // LOG(INFO) << "Motion intrinsics: {" << api->GetMotionIntrinsics() << "}";
  // LOG(INFO) << "Motion extrinsics left to imu: {" << api->GetMotionExtrinsics(mynteye::Stream::LEFT) << "}";

  // NOTE The hardware stores the previous settings, so not setting the options
  // may NOT give the defaults.

  // FRAME_RATE values: 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60
  api->SetOptionValue(mynteye::Option::FRAME_RATE, 30);
  // IMU_FREQUENCY values: 100, 200, 250, 333, 500
  api->SetOptionValue(mynteye::Option::IMU_FREQUENCY, 500);

  // ACCELEROMETER_RANGE values: 4, 8, 16, 32
  api->SetOptionValue(mynteye::Option::ACCELEROMETER_RANGE, 8);
  // GYROSCOPE_RANGE values: 500, 1000, 2000, 4000
  api->SetOptionValue(mynteye::Option::GYROSCOPE_RANGE, 1000);

  // 0 = auto-exposure, 1 = manual
  api->SetOptionValue(mynteye::Option::EXPOSURE_MODE, 0);

  if (api->GetOptionValue(mynteye::Option::EXPOSURE_MODE) == 0) {
    // max_gain: range [0,48], default 48
    api->SetOptionValue(mynteye::Option::MAX_GAIN, 48);
    // max_exposure_time: range [0,240], default 240
    api->SetOptionValue(mynteye::Option::MAX_EXPOSURE_TIME, 240);
    // min_exposure_time: range [0,240], default 0
    // api->SetOptionValue(mynteye::Option::MIN_EXPOSURE_TIME, 0);
    // desired_brightness: range [0,255], default 192
    api->SetOptionValue(mynteye::Option::DESIRED_BRIGHTNESS, 192);
  }
  else {
    // gain: range [0,48], default 24
    api->SetOptionValue(mynteye::Option::GAIN, 24);
    // brightness/exposure_time: range [0,240], default 120
    api->SetOptionValue(mynteye::Option::BRIGHTNESS, 120);
    // contrast/black_level_calibration: range [0,254], default 116
    api->SetOptionValue(mynteye::Option::CONTRAST, 116);
  }

  mynteye::Model model = api->GetModel();
  if (model == mynteye::Model::STANDARD) {
    // ir control: range [0,160], default 0
    api->SetOptionValue(mynteye::Option::IR_CONTROL, 0);
  }

  // auto request = api->GetStreamRequest();
  // api->ConfigStreamRequest(request);

  std::cout << std::endl;
  LOG(INFO) << "FRAME_RATE [10,60]Hz: " << api->GetOptionValue(mynteye::Option::FRAME_RATE);
  LOG(INFO) << "IMU_FREQUENCY [100,500]Hz: " << api->GetOptionValue(mynteye::Option::IMU_FREQUENCY);
  LOG(INFO) << "ACCELEROMETER_RANGE [4, 32]g: " << api->GetOptionValue(mynteye::Option::ACCELEROMETER_RANGE);
  LOG(INFO) << "GYROSCOPE_RANGE [500, 4000]deg/s: " << api->GetOptionValue(mynteye::Option::GYROSCOPE_RANGE);
  if (api->GetOptionValue(mynteye::Option::EXPOSURE_MODE) == 0) {
    LOG(INFO) << "auto-exposure enabled:";
    LOG(INFO) << "  MAX_GAIN [0,48]: " << api->GetOptionValue(mynteye::Option::MAX_GAIN);
    LOG(INFO) << "  MAX_EXPOSURE_TIME [0,240]: " << api->GetOptionValue(mynteye::Option::MAX_EXPOSURE_TIME);
    LOG(INFO) << "  MIN_EXPOSURE_TIME [0,240]: " << api->GetOptionValue(mynteye::Option::MIN_EXPOSURE_TIME);
    LOG(INFO) << "  DESIRED_BRIGHTNESS [0,255]: " << api->GetOptionValue(mynteye::Option::DESIRED_BRIGHTNESS);
  }
  else {
    LOG(INFO) << "manual exposure enabled:";
    LOG(INFO) << "  GAIN [0,48]: " << api->GetOptionValue(mynteye::Option::GAIN);
    LOG(INFO) << "  BRIGHTNESS [0,240]: " << api->GetOptionValue(mynteye::Option::BRIGHTNESS);
    LOG(INFO) << "  CONTRAST [0,254]: " << api->GetOptionValue(mynteye::Option::CONTRAST);
  }
  if (api->GetOptionValue(mynteye::Option::IR_CONTROL) > 0) {
    LOG(INFO) << "IR enabled [0,160]: " << api->GetOptionValue(mynteye::Option::IR_CONTROL);
  }
  else {
    LOG(INFO) << "IR disabled";
  }

  // Sorts IMU and frames by timestamps, but causes lag and makes video preview less smooth (why?).
  // api->EnableTimestampCorrespondence(mynteye::Stream::LEFT);

  api->EnableMotionDatas();

  // api->DisableStreamData(mynteye::Stream::LEFT);
  // api->DisableStreamData(mynteye::Stream::RIGHT);
  // api->EnableStreamData(mynteye::Stream::LEFT_RECTIFIED);
  // api->EnableStreamData(mynteye::Stream::RIGHT_RECTIFIED);

  return api;
}

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);
  std::shared_ptr<mynteye::API> api = setupMynt(argc, argv);

  std::string outputDir = "output/" + currentISO8601TimeUTC();
  system(("mkdir -p " + outputDir + "/").c_str());
  std::unique_ptr<recorder::Recorder> recorder = recorder::Recorder::build(
     outputDir + "/data.jsonl",
     outputDir + "/data.avi");

  api->Start(mynteye::Source::ALL);

  cv::namedWindow("frame");

  // Constant for duration of recording.
  mynteye::IntrinsicsEquidistant intrinsics[2] = {
    api->GetIntrinsics<mynteye::IntrinsicsEquidistant>(mynteye::Stream::LEFT),
    api->GetIntrinsics<mynteye::IntrinsicsEquidistant>(mynteye::Stream::RIGHT)
  };
  // LOG(INFO) << intrinsics[0];
  // LOG(INFO) << intrinsics[1];

  for (int index = 0; index < 2; index++) {
    nlohmann::json lensMetadata;
    lensMetadata["model"] = "KANNALA_BRANDT4";
    auto coeffs = nlohmann::json::array();
    for(auto i = 0; i < 4; ++i) {
      coeffs.push_back(intrinsics[index].coeffs[i]);
    }
    lensMetadata["coeffs"] = coeffs;
    lensMetadata["cameraInd"] = index;
    lensMetadata["focalLengthX"] = intrinsics[index].coeffs[4];
    lensMetadata["focalLengthY"] = intrinsics[index].coeffs[5];
    lensMetadata["px"] = intrinsics[index].coeffs[6];
    lensMetadata["py"] = intrinsics[index].coeffs[7];
    recorder->addJson(lensMetadata);
  }

  cv::Mat colorFrames[2];
  cv::Mat preview;

  std::cout << std::endl;
  LOG(INFO) << "Press Enter to end recording.";

  auto startTime = std::chrono::steady_clock::now();
  std::size_t imgCount = 0;
  std::size_t imuCount = 0;
  while (true) {
    api->WaitForStreams();

    auto &&left = api->GetStreamData(mynteye::Stream::LEFT);
    auto &&right = api->GetStreamData(mynteye::Stream::RIGHT);
    // auto &&left = api->GetStreamData(mynteye::Stream::LEFT_RECTIFIED);
    // auto &&right = api->GetStreamData(mynteye::Stream::RIGHT_RECTIFIED);

    assert(left.frame_raw->format() == mynteye::Format::GREY);
    assert(right.frame_raw->format() == mynteye::Format::GREY);

    std::vector<recorder::FrameData> frameGroup;
    if (!left.frame.empty() && !right.frame.empty()) {
      cv::hconcat(left.frame, right.frame, preview);
      cv::imshow("preview", preview);

      for (int index = 0; index < 2; index++) {
        const auto &s = index == 0 ? left : right;
        cv::cvtColor(s.frame, colorFrames[index], cv::COLOR_GRAY2BGR);
        // coeffs[8] = k2,k3,k4,k5,mu,mv,u0,v0
        // Save focal length and principal point per frame even though they are constant.
        recorder::FrameData frameData({
            .t = 1e-6 * s.img->timestamp,
            .cameraInd = index,
            .focalLength = 0.5 * (intrinsics[index].coeffs[4] + intrinsics[index].coeffs[5]),
            // .focalLengthX = intrinsics[index].coeffs[4],
            // .focalLengthY = intrinsics[index].coeffs[5],
            .px = intrinsics[index].coeffs[6],
            .py = intrinsics[index].coeffs[7],
            .frameData = colorFrames + index,
        });
        frameGroup.push_back(frameData);
      }
      recorder->addFrameGroup(frameGroup[0].t, frameGroup);
      imgCount++;
    }
    else {
      LOG(WARNING) << "No stereo image.";
    }

    auto &&motion_datas = api->GetMotionDatas();
    imuCount += motion_datas.size();

    for (auto &&data : motion_datas) {
      if (data.imu->flag != 0) {
        LOG(WARNING) << "Invalid gyroscope or accelerometer measurement.";
        continue;
      }
      double t = 1e-6 * data.imu->timestamp;
      constexpr double DEG_TO_RADIAN = 2 * M_PI / 360.;
      recorder->addGyroscope(t,
          DEG_TO_RADIAN * data.imu->gyro[0],
          DEG_TO_RADIAN * data.imu->gyro[1],
          DEG_TO_RADIAN * data.imu->gyro[2]);
      // NOTE Not sure if this is correct, should check if Mynt documents the
      // `g` value somewhere. Apple for example uses exactly 9.81.
      // <https://en.wikipedia.org/wiki/Standard_gravity>
      constexpr double G = 9.80665;
      recorder->addAccelerometer(t,
          G * data.imu->accel[0],
          G * data.imu->accel[1],
          G * data.imu->accel[2]);
      // Also record `data.imu->temperature`?
    }

    // Quit with Escape, Enter or Q.
    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 13 || key == 'q' || key == 'Q') {
      break;
    }
  }

  const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - startTime).count();

  api->Stop(mynteye::Source::ALL);

  std::cout << std::endl;
  LOG(INFO) << "Camera FPS: " << (1000. * imgCount / elapsed);
  LOG(INFO) << "Imu frequency: " << (1000. * imuCount / elapsed) << " Hz";
  std::cout << std::endl;

  return 0;
}
