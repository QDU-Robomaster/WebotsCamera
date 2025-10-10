#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Webots simulated camera publisher (RGB8)
constructor_args:
  - info:
      width: 1280
      height: 720
      step: 3840
      timestamp: 0
      encoding: CameraBase::Encoding::RGB8
      camera_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
      distortion_model: CameraBase::DistortionModel::PLUMB_BOB
      distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
      rectification_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      projection_matrix: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - runtime:
      device_name: "camera"   # Webots 场景树中的 Camera 名称
      fps: 30                 # 期望帧率（通过 enable 周期近似）
      exposure: 1.0           # Webots 相机曝光比例（>1 更亮，<1 更暗）
      gain: 0.0               # 保留字段：Webots 无原生增益，记录+告警
template_args: []
required_hardware: []
depends:
  - qdu-future/CameraBase
=== END MANIFEST === */
// clang-format on

// STL / OpenCV
#include <array>
#include <atomic>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <string>

// LibXR
#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"

// Webots
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>

class WebotsCamera : public LibXR::Application, public CameraBase
{
 public:
  static constexpr int MAX_W = 4096;
  static constexpr int MAX_H = 3072;
  static constexpr int CH = 3;  // RGB8
  static constexpr size_t BUF_BYTES = static_cast<size_t>(MAX_W) * MAX_H * CH;

  struct RuntimeParam
  {
    std::string device_name = "camera";
    int fps = 30;  // 采样周期 = round(1000/fps) ms（Webots 按毫秒周期采样）
    double exposure = 1.0;  // Webots 相机曝光比例（下一帧生效）
    double gain = 0.0;      // 占位：Webots 无原生增益（仅记录）
  };

 public:
  // 与 Hik/Uvc 保持一致的构造签名
  explicit WebotsCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                        const CameraBase::CameraInfo info, const RuntimeParam runtime);

  ~WebotsCamera();

  void SetRuntimeParam(const RuntimeParam& p);
  void OnMonitor() override {}

  // === CameraBase 纯虚接口实现 ===
  void SetExposure(double exposure) override;  // RamFS: set_exposure <val>
  void SetGain(double gain) override;          // RamFS: set_gain <val>（记录+告警）

 private:
  void UpdateParameters();
  static void ThreadFun(WebotsCamera* self);
  static int fps_to_period_ms(int fps, int fallback_ms);

 private:
  // 大 RGB 缓冲（匹配其它相机模块风格）
  std::unique_ptr<std::array<uint8_t, BUF_BYTES>> frame_buf_{};

  // 参数
  CameraBase::CameraInfo info_{};
  RuntimeParam runtime_{};

  // Topics（与其它相机保持一致的名字/类型）
  LibXR::Topic frame_topic_ = LibXR::Topic("image_raw", sizeof(cv::Mat));
  LibXR::Topic info_topic_ = LibXR::Topic("camera_info", sizeof(CameraBase::CameraInfo));

  // Webots 设备
  webots::Robot* robot_{};
  webots::Camera* cam_ = nullptr;
  int time_step_ms_ = 0;       // 世界 basicTimeStep（ms）
  int sample_period_ms_ = 33;  // Camera enable 周期 ~ 1000/fps

  // 线程 / 状态
  std::atomic<bool> running_{false};
  LibXR::Thread capture_thread_{};
  int fail_count_ = 0;
};

// 由宿主环境提供的 Webots Robot 指针（与现有代码保持一致）
extern webots::Robot* _libxr_webots_robot_handle;
