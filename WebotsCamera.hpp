#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Webots simulated camera publisher (BGR8)
constructor_args:
  - runtime:
      device_name: "camera"   # Webots 场景树中的 Camera 名称
      fps: 30                 # 期望帧率（通过 enable 周期近似）
      exposure: 1.0           # Webots 相机曝光比例（>1 更亮，<1 更暗）
      gain: 0.0               # 保留字段：Webots 无原生增益，记录+告警
template_args:
  - Info:
      width: 1280
      height: 720
      step: 3840
      encoding: CameraBase::Encoding::BGR8
      camera_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      distortion_model: CameraBase::DistortionModel::PLUMB_BOB
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
#include <memory>

#include <opencv2/core/mat.hpp>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "thread.hpp"

#include <webots/Camera.hpp>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

template <CameraBase::CameraInfo CameraInfoV>
class WebotsCamera : public LibXR::Application, public CameraBase
{
 public:
  static inline constexpr CameraBase::CameraInfo kCameraInfo = CameraInfoV;
  static constexpr int MAX_W = 4096;
  static constexpr int MAX_H = 3072;
  static constexpr int CH = 3;
  static constexpr size_t BUF_BYTES = static_cast<size_t>(MAX_W) * MAX_H * CH;

  struct RuntimeParam
  {
    const char* device_name = "camera";
    int fps = 30;
    double exposure = 1.0;
    double gain = 0.0;
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
  CameraBase::PoseStamped ReadCameraPoseStamped(LibXR::MicrosecondTimestamp timestamp) const;

 private:
  std::unique_ptr<std::array<uint8_t, BUF_BYTES>> frame_buf_{};

  RuntimeParam runtime_{};

  LibXR::Topic frame_topic_ = LibXR::Topic("image_raw", sizeof(cv::Mat));
  LibXR::Topic image_header_topic_ =
      LibXR::Topic("image_header", sizeof(CameraBase::ImageHeader));
  LibXR::Topic camera_pose_topic_ =
      LibXR::Topic("camera_pose", sizeof(CameraBase::PoseStamped));
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
