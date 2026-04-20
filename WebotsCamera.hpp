#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Webots simulated camera publisher (BGR8)
constructor_args:
  - runtime:
      device_name: "camera"
      fps: 30
      exposure: 1.0
      gain: 0.0
      image_topic_name: "image_frame"
template_args:
  - Info:
      width: 1280
      height: 720
      step: 3840
      encoding: CameraTypes::Encoding::BGR8
      camera_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      distortion_model: CameraTypes::DistortionModel::PLUMB_BOB
      distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
required_hardware: []
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cstdint>

#include <opencv2/core/mat.hpp>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "linux_shared_topic.hpp"
#include "message.hpp"
#include "thread.hpp"

#include <webots/Camera.hpp>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

template <CameraTypes::CameraInfo CameraInfoV>
class WebotsCamera : public LibXR::Application, public CameraBase<CameraInfoV>
{
 public:
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using SharedImageFrame = typename Base::SharedImageFrame;
  using SharedImageTopic = LibXR::LinuxSharedTopic<SharedImageFrame>;

  static inline constexpr CameraInfo kCameraInfo = CameraInfoV;
  static constexpr int CH = 3;
  static constexpr LibXR::LinuxSharedTopicConfig kImageTopicConfig{
      .slot_num = 4,
      .subscriber_num = 2,
      .queue_num = 4,
  };

  struct PoseStamped
  {
    LibXR::MicrosecondTimestamp timestamp{};
    LibXR::Quaternion<float> rotation{};
    LibXR::Position<float> translation{};
  };

  struct RuntimeParam
  {
    const char* device_name = "camera";
    int fps = 30;
    double exposure = 1.0;
    double gain = 0.0;
    const char* image_topic_name = "image_frame";
  };

 public:
  explicit WebotsCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                        RuntimeParam runtime);

  ~WebotsCamera();

  void SetRuntimeParam(const RuntimeParam& p);
  void OnMonitor() override {}

  void SetExposure(double exposure) override;
  void SetGain(double gain) override;

 private:
  void UpdateParameters();
  static void ThreadFun(WebotsCamera<CameraInfoV>* self);
  static int FpsToPeriodMS(int fps, int fallback_ms);
  PoseStamped ReadCameraPoseStamped(LibXR::MicrosecondTimestamp timestamp) const;

 private:
  RuntimeParam runtime_{};
  SharedImageTopic image_frame_topic_;
  LibXR::Topic camera_pose_topic_ = LibXR::Topic("camera_pose", sizeof(PoseStamped));
  LibXR::Topic gimbal_rotation_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation");

  webots::Robot* robot_{};
  webots::Camera* cam_ = nullptr;
  webots::Node* cam_node_ = nullptr;
  webots::Supervisor* supervisor_ = nullptr;
  int time_step_ms_ = 0;
  int sample_period_ms_ = 33;
  int publish_interval_steps_ = 1;

  std::atomic<bool> running_{false};
  LibXR::Thread capture_thread_{};
  int fail_count_ = 0;
  uint64_t frame_sequence_ = 0;
  uint64_t last_publish_bucket_ = UINT64_MAX;
};

extern webots::Robot* _libxr_webots_robot_handle;

#include "WebotsCamera.inl"
