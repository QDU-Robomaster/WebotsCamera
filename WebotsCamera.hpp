#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Webots 仿真相机：按真车几何输出 BayerRG8，发布 IMU，并作为触发 GPIO / Webots camera that outputs BayerRG8 in the robot camera geometry, publishes the IMU and acts as the trigger GPIO
depends:
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <Eigen/Core>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Robot.hpp>

#include "CameraBase.hpp"
#include "WebotsBayer.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "transform.hpp"

extern webots::Robot* _libxr_webots_robot_handle;

/// 仿真相机设置，与 YAML 一一对应 / Simulated camera settings, one-to-one with the YAML.
struct WebotsCameraSettings
{
  double exposure;              ///< Webots Camera 曝光 / Webots Camera exposure
  std::string_view gyro_topic;  ///< 与 CameraFrameSync 配置的 IMU Topic 名相同
  std::string_view accl_topic;  ///< Same IMU Topic names as configured in CameraFrameSync
  std::string_view quat_topic;
};

/**
 * @brief Webots 仿真相机。Webots Camera 按原生分辨率渲染，每帧按当前视角在软件里取窗、
 *        抽样成 BayerRG8，下游看到的帧与真车相同。它同时模拟 MCU 的两项工作：每个仿真步
 *        发布 IMU，并作为 CameraSync 的触发 GPIO。
 *        Webots camera. The Webots Camera renders at native resolution; each frame is
 *        windowed for the current view and sampled into BayerRG8 in software, so
 *        downstream sees the robot's frames. It also plays two MCU roles: it publishes
 *        the IMU every simulation step and is the trigger GPIO for CameraSync.
 *
 * Webots 设备：相机 `<name>`，IMU `<name>_gyro`、`<name>_accelerometer`、
 * `<name>_inertial_unit`。上升沿触发一帧：下一个仿真步打开相机，再下一步取图并关闭相机，
 * 帧时间为取图时的仿真时间（比触发晚一个仿真步）。帧计数每个边沿加一、切档后从 0 起。
 * Webots devices: camera `<name>`, IMU `<name>_gyro`, `<name>_accelerometer`,
 * `<name>_inertial_unit`. A rising edge triggers one frame: the camera is enabled on
 * the next step and read and disabled on the following one; the frame time is the
 * simulation time of the read, one step after the edge. The frame counter advances per
 * edge and restarts at 0 after a view switch.
 */
class WebotsCamera : public CameraBase, public LibXR::GPIO
{
 public:
  using ImuVector = Eigen::Matrix<float, 3, 1>;
  using ImuQuaternion = LibXR::Quaternion<float>;

  WebotsCamera(const CameraTypes::CameraCalibration& calibration, NarrowPosition narrow,
               std::string_view name, const WebotsCameraSettings& settings)
      : CameraBase(calibration, narrow, name, SlotPolicy::DROP),
        robot_(_libxr_webots_robot_handle),
        gyro_topic_(LibXR::Topic::CreateTopic<ImuVector>(
            std::string(settings.gyro_topic).c_str())),
        accl_topic_(LibXR::Topic::CreateTopic<ImuVector>(
            std::string(settings.accl_topic).c_str())),
        quat_topic_(LibXR::Topic::CreateTopic<ImuQuaternion>(
            std::string(settings.quat_topic).c_str())),
        window_(CurrentGeometry())
  {
    REQUIRE(robot_ != nullptr);
    camera_ = robot_->getCamera(Name());
    gyro_ = robot_->getGyro(Name() + "_gyro");
    accelerometer_ = robot_->getAccelerometer(Name() + "_accelerometer");
    inertial_unit_ = robot_->getInertialUnit(Name() + "_inertial_unit");
    if (camera_ == nullptr || gyro_ == nullptr || accelerometer_ == nullptr ||
        inertial_unit_ == nullptr)
    {
      XR_LOG_ERROR(
          "%s: Webots devices %s, %s_gyro, %s_accelerometer, %s_inertial_unit "
          "are required",
          Name().c_str(), Name().c_str(), Name().c_str(), Name().c_str(), Name().c_str());
      REQUIRE(false);
    }
    REQUIRE(WorldMatchesCalibration());
    step_ms_ = static_cast<int>(std::lround(robot_->getBasicTimeStep()));
    camera_->setExposure(settings.exposure);
    gyro_->enable(step_ms_);
    accelerometer_->enable(step_ms_);
    inertial_unit_->enable(step_ms_);

    running_.store(true);
    step_thread_.Create<WebotsCamera*>(this, StepThread, "webots_camera",
                                       STEP_STACK_BYTES,
                                       LibXR::Thread::Priority::REALTIME);
    StartCapture();
  }

  /// 步进线程随仿真进程结束，析构只停采集线程 / The step thread ends with the
  /// simulation process; destruction stops only the capture thread.
  ~WebotsCamera() override
  {
    running_.store(false);
    StopCapture();
  }

  /// 触发电平：上升沿触发一帧 / Trigger level: a rising edge triggers one frame.
  void Write(bool level) override
  {
    const bool was_high = level_.exchange(level);
    if (level && !was_high)
    {
      edges_.fetch_add(1);
    }
  }
  bool Read() override { return level_.load(); }
  LibXR::ErrorCode EnableInterrupt() override { return LibXR::ErrorCode::OK; }
  LibXR::ErrorCode DisableInterrupt() override { return LibXR::ErrorCode::OK; }
  LibXR::ErrorCode SetConfig(Configuration) override { return LibXR::ErrorCode::OK; }

 private:
  static constexpr std::size_t STEP_STACK_BYTES = 256 * 1024;
  /// 等待时检查是否停止的间隔 / How often waits check for a stop request.
  static constexpr auto WAIT_STEP = std::chrono::milliseconds(10);

  /// 世界里的相机须与标定一致：分辨率、由视场角换算的焦距、主点在中心。
  /// The world camera must match the calibration: resolution, focal length from the
  /// field of view, principal point at the centre.
  bool WorldMatchesCalibration() const
  {
    const CameraTypes::CameraCalibration& c = Calibration();
    const double width = camera_->getWidth();
    const double height = camera_->getHeight();
    const double fx = width / 2.0 / std::tan(camera_->getFov() / 2.0);
    const bool ok =
        width == c.native_width && height == c.native_height &&
        std::abs(c.fx - fx) <= 0.005 * fx && std::abs(c.fy - fx) <= 0.005 * fx &&
        std::abs(c.cx - width / 2.0) <= 1.0 && std::abs(c.cy - height / 2.0) <= 1.0;
    if (!ok)
    {
      XR_LOG_ERROR(
          "%s: world camera %.0fx%.0f fx=%.1f does not match the calibration "
          "%ux%u fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
          Name().c_str(), width, height, fx, c.native_width, c.native_height, c.fx, c.fy,
          c.cx, c.cy);
    }
    return ok;
  }

  /// 采集线程：等步进线程交出一帧 / Capture thread: wait for the step thread's frame.
  bool GrabFrame(ImageFrame& frame) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!ready_)
    {
      if (!CaptureRunning())
      {
        return false;
      }
      ready_cv_.wait_for(lock, WAIT_STEP);
    }
    ready_ = false;
    frame.data = buffer_;
    frame.timestamp_us = buffer_timestamp_;
    frame.frame_counter = buffer_counter_;
    frame.geometry = buffer_geometry_;
    return true;
  }

  /// 只改软件取窗，立即生效；帧计数从 0 起，丢掉未取走的帧。
  /// Changes only the software window, effective at once; the counter restarts at 0
  /// and a frame not yet taken is dropped.
  LibXR::ErrorCode ApplyView(const CameraTypes::FrameGeometry& geometry) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    window_ = geometry;
    counter_base_ = edges_.load();
    ready_ = false;
    return LibXR::ErrorCode::OK;
  }

  /// 只改软件取窗，立即生效；之后渲染的帧带新窗口 / Changes only the software window,
  /// effective at once; frames rendered afterwards carry the new window.
  LibXR::ErrorCode ApplyOffset(const CameraTypes::FrameGeometry& geometry) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    window_ = geometry;
    return LibXR::ErrorCode::OK;
  }

  static void StepThread(WebotsCamera* self)
  {
    uint64_t last_step = UINT64_MAX;
    while (self->running_.load())
    {
      LibXR::Thread::Sleep(self->step_ms_);
      const uint64_t now_us =
          static_cast<uint64_t>(std::llround(self->robot_->getTime() * 1e6));
      const uint64_t step = now_us / (static_cast<uint64_t>(self->step_ms_) * 1000);
      if (step != last_step)
      {
        last_step = step;
        self->OnStep(now_us);
      }
    }
  }

  /// 每个仿真步：发 IMU；取上一步打开的图；有新边沿则打开相机。
  /// Every step: publish the IMU, read the image enabled on the previous step, and
  /// enable the camera for a new edge.
  void OnStep(uint64_t now_us)
  {
    PublishImu(now_us);
    if (rendering_)
    {
      const uint8_t* bgra = camera_->getImage();
      std::lock_guard<std::mutex> lock(mutex_);
      // 切档前边沿的帧丢掉，与真车停流时一样 / A frame of an edge before the view
      // switch is dropped, as when the robot camera stops streaming.
      if (bgra != nullptr && render_edge_ >= counter_base_)
      {
        WebotsBayer::Render(bgra, Calibration().native_width, window_, buffer_);
        buffer_timestamp_ = LibXR::MicrosecondTimestamp(now_us);
        buffer_counter_ = render_edge_ - counter_base_;
        buffer_geometry_ = window_;
        ready_ = true;
        ready_cv_.notify_one();
      }
      camera_->disable();
      rendering_ = false;
    }
    const uint32_t edges = edges_.load();
    if (edges != handled_edges_)
    {
      handled_edges_ = edges;
      render_edge_ = edges - 1;  // 只渲染最新的边沿 / Render only the latest edge
      camera_->enable(step_ms_);
      rendering_ = true;
    }
  }

  /// Webots IMU 转成公共机体系（x 右、y 前、z 上）后发布 / Publish the Webots IMU in the
  /// body frame (x right, y forward, z up).
  void PublishImu(uint64_t now_us)
  {
    const double* w = gyro_->getValues();
    const double* a = accelerometer_->getValues();
    const double* q = inertial_unit_->getQuaternion();  // x, y, z, w
    const LibXR::MicrosecondTimestamp timestamp(now_us);
    // Webots 传感器系绕 z 转 180° 得到机体系 / Body frame = sensor frame turned 180°
    // about z.
    ImuVector gyro(static_cast<float>(-w[0]), static_cast<float>(-w[1]),
                   static_cast<float>(w[2]));
    ImuVector accl(static_cast<float>(-a[0]), static_cast<float>(-a[1]),
                   static_cast<float>(a[2]));
    const ImuQuaternion sensor(static_cast<float>(q[3]), static_cast<float>(q[0]),
                               static_cast<float>(q[1]), static_cast<float>(q[2]));
    const ImuQuaternion turn(0.0F, 0.0F, 0.0F, 1.0F);
    const ImuQuaternion turn_back(0.0F, 0.0F, 0.0F, -1.0F);
    ImuQuaternion quat = turn * sensor * turn_back;
    gyro_topic_.Publish(gyro, timestamp);
    accl_topic_.Publish(accl, timestamp);
    quat_topic_.Publish(quat, timestamp);
  }

  webots::Robot* robot_;
  webots::Camera* camera_ = nullptr;
  webots::Gyro* gyro_ = nullptr;
  webots::Accelerometer* accelerometer_ = nullptr;
  webots::InertialUnit* inertial_unit_ = nullptr;
  LibXR::Topic gyro_topic_;
  LibXR::Topic accl_topic_;
  LibXR::Topic quat_topic_;
  int step_ms_ = 1;
  LibXR::Thread step_thread_;
  std::atomic<bool> running_{false};

  // 触发 / Trigger.
  std::atomic<bool> level_{false};
  std::atomic<uint32_t> edges_{0};
  uint32_t handled_edges_ = 0;  // 步进线程 / Step thread only
  uint32_t render_edge_ = 0;    // 步进线程 / Step thread only
  bool rendering_ = false;      // 步进线程 / Step thread only

  // 步进线程交给采集线程的一帧，由 mutex_ 保护 / One frame handed from the step thread
  // to the capture thread, guarded by mutex_.
  std::mutex mutex_;
  std::condition_variable ready_cv_;
  CameraTypes::FrameGeometry window_;
  uint32_t counter_base_ = 0;
  bool ready_ = false;
  std::array<uint8_t, CameraTypes::FRAME_BYTES> buffer_{};
  LibXR::MicrosecondTimestamp buffer_timestamp_{};
  uint32_t buffer_counter_ = 0;
  CameraTypes::FrameGeometry buffer_geometry_{};
};
