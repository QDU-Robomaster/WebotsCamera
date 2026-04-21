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
      pose_def_name: "camera"
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

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "libxr_system.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "transform.hpp"

#include <webots/Camera.hpp>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

extern webots::Robot* _libxr_webots_robot_handle;

/**
 * @brief Webots 相机模块。
 *
 * 该模块负责四件事：
 * 1. 从 Webots Camera 取出原始 BGRA 图像；
 * 2. 按目标 fps 把发布节奏量化到世界步长；
 * 3. 从 supervisor 读取相机 pose，并由连续 pose 估计 motion；
 * 4. 发布 imu，再把图像写入 `CameraBase::ImageFrame` 并交给已注册的图像 sink。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class WebotsCamera : public LibXR::Application, public CameraBase<CameraInfoV>
{
 public:
  using Self = WebotsCamera<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using ImageFrame = typename Base::ImageFrame;
  using ImuStamped = typename Base::ImuStamped;

  static inline constexpr CameraInfo camera_info = Base::camera_info;
  static constexpr int channel_count = 3;
  static constexpr int frame_width = static_cast<int>(camera_info.width);
  static constexpr int frame_height = static_cast<int>(camera_info.height);
  static constexpr std::size_t frame_step = static_cast<std::size_t>(camera_info.step);

  static_assert(camera_info.encoding == CameraTypes::Encoding::BGR8,
                "WebotsCamera requires BGR8 output encoding");
  static_assert(frame_step == static_cast<std::size_t>(camera_info.width) * channel_count,
                "WebotsCamera requires packed BGR step");

  struct PoseSample
  {
    LibXR::MicrosecondTimestamp timestamp{};
    LibXR::Quaternion<float> rotation{};
    LibXR::Position<float> translation{};
  };

  struct MotionSample
  {
    LibXR::Position<float> angular_velocity{};
    LibXR::Position<float> linear_acceleration{};
  };

  struct RuntimeParam
  {
    // Webots Camera 设备名。
    const char* device_name = "camera";

    // 目标发布频率。最终会被量化到世界步长上。
    int fps = 30;

    // Webots 原生支持曝光，不支持 gain。
    double exposure = 1.0;
    double gain = 0.0;

    // 用于 supervisor 查询相机位姿的 DEF 名字。
    std::string pose_def_name = "camera";

    // 图像与 imu 的原始输出名，供同步器和其他消费者复用。
    const char* image_topic_name = "camera_image";
    const char* imu_topic_name = "camera_imu";
  };

  explicit WebotsCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                        RuntimeParam runtime)
      : Base(hw, runtime.device_name, runtime.image_topic_name, runtime.imu_topic_name),
        runtime_(std::move(runtime)),
        robot_(_libxr_webots_robot_handle)
  {
    XR_LOG_INFO("Starting WebotsCamera!");

    InitRobot();
    InitCamera();
    InitSupervisor(hw);
    ValidateCameraGeometry();
    ConfigureSamplingOnStartup();
    ApplyExposure();
    WarnUnsupportedGainIfNeeded();
    StartCaptureThread();

    XR_LOG_PASS(
        "Webots camera enabled: name=%s, period=%d ms, world_dt=%d ms, publish_every=%d step(s)",
        runtime_.device_name, sample_period_ms_, time_step_ms_, publish_interval_steps_);

    app.Register(*this);
  }

  ~WebotsCamera()
  {
    running_.store(false);

    if (cam_ != nullptr)
    {
      cam_->disable();
      cam_ = nullptr;
    }

    XR_LOG_INFO("WebotsCamera destroyed!");
  }

  void SetRuntimeParam(const RuntimeParam& runtime)
  {
    const bool pose_def_changed = runtime_.pose_def_name != runtime.pose_def_name;
    const bool device_changed = !SameCStr(runtime_.device_name, runtime.device_name);

    RuntimeParam next = runtime;
    if (device_changed)
    {
      XR_LOG_WARN(
          "WebotsCamera: device_name runtime change '%s' -> '%s' requires reconstruction. "
          "Keeping current camera binding.",
          SafeCStr(runtime_.device_name), SafeCStr(runtime.device_name));
      next.device_name = runtime_.device_name;
    }

    runtime_ = std::move(next);
    if (pose_def_changed)
    {
      cam_node_ = nullptr;
    }

    ApplyRuntimeParameters();
  }

  void OnMonitor() override {}

  void SetExposure(double exposure) override
  {
    runtime_.exposure = exposure;
    ApplyExposure();
  }

  void SetGain(double gain) override
  {
    runtime_.gain = gain;
    WarnUnsupportedGainIfNeeded();
  }

 private:
  // ---- 基础工具 ----

  static const char* SafeCStr(const char* text) { return text != nullptr ? text : ""; }

  static bool SameCStr(const char* lhs, const char* rhs)
  {
    return std::strcmp(SafeCStr(lhs), SafeCStr(rhs)) == 0;
  }

  static int FpsToPeriodMs(int fps, int fallback_ms)
  {
    if (fps <= 0)
    {
      return fallback_ms;
    }

    const double period = 1000.0 / static_cast<double>(fps);
    const int ms = static_cast<int>(std::lround(period));
    return ms < 1 ? 1 : ms;
  }

  static int ComputePublishIntervalSteps(int period_ms, int time_step_ms)
  {
    return std::max(1, static_cast<int>(std::lround(static_cast<double>(period_ms) /
                                                    static_cast<double>(time_step_ms))));
  }

  // ---- 启动与运行时参数 ----

  void InitRobot()
  {
    if (robot_ == nullptr)
    {
      XR_LOG_ERROR("Webots robot handle is null!");
      std::exit(-1);
    }

    time_step_ms_ = static_cast<int>(robot_->getBasicTimeStep());
    if (time_step_ms_ <= 0)
    {
      XR_LOG_ERROR("Webots basic timestep is invalid: %d ms", time_step_ms_);
      throw std::runtime_error("WebotsCamera: invalid basic timestep");
    }
  }

  void InitCamera()
  {
    cam_ = robot_->getCamera(runtime_.device_name);
    if (cam_ == nullptr)
    {
      XR_LOG_ERROR("Webots Camera '%s' not found!", runtime_.device_name);
      throw std::runtime_error("WebotsCamera: camera device not found");
    }
  }

  void ValidateCameraGeometry() const
  {
    const uint32_t actual_width = static_cast<uint32_t>(cam_->getWidth());
    const uint32_t actual_height = static_cast<uint32_t>(cam_->getHeight());
    const uint32_t packed_step = actual_width * channel_count;

    if (camera_info.width == actual_width && camera_info.height == actual_height &&
        camera_info.step == packed_step)
    {
      return;
    }

    XR_LOG_ERROR(
        "WebotsCamera: constexpr geometry mismatch width=%u/%u height=%u/%u step=%u/%u",
        camera_info.width, actual_width, camera_info.height, actual_height,
        camera_info.step, packed_step);
    throw std::runtime_error("WebotsCamera: constexpr geometry mismatch");
  }

  void ConfigureSamplingOnStartup()
  {
    sample_period_ms_ = FpsToPeriodMs(runtime_.fps, 33);
    publish_interval_steps_ = ComputePublishIntervalSteps(sample_period_ms_, time_step_ms_);
    last_publish_bucket_ = UINT64_MAX;
    cam_->enable(sample_period_ms_);
  }

  void InitSupervisor(LibXR::HardwareContainer& hw)
  {
    supervisor_ = hw.FindOrExit<webots::Supervisor>({"supervisor"});
  }

  void StartCaptureThread()
  {
    running_.store(true);
    capture_thread_.Create(this, ThreadFun, "WebotsCameraThread",
                           static_cast<size_t>(1024 * 128),
                           LibXR::Thread::Priority::REALTIME);
  }

  void ApplyRuntimeParameters()
  {
    if (cam_ == nullptr)
    {
      return;
    }

    ReconfigureSampling();
    ApplyExposure();
    WarnUnsupportedGainIfNeeded();
  }

  void ReconfigureSampling()
  {
    const int new_period = FpsToPeriodMs(runtime_.fps, sample_period_ms_);
    const int new_publish_interval =
        ComputePublishIntervalSteps(new_period, time_step_ms_);

    if (new_period != sample_period_ms_)
    {
      sample_period_ms_ = new_period;
      cam_->disable();
      cam_->enable(sample_period_ms_);
      XR_LOG_INFO("WebotsCamera: fps target=%d -> period=%d ms", runtime_.fps,
                  sample_period_ms_);
    }

    publish_interval_steps_ = new_publish_interval;
    last_publish_bucket_ = UINT64_MAX;
  }

  void ApplyExposure()
  {
    if (cam_ == nullptr)
    {
      XR_LOG_WARN("WebotsCamera: camera not ready yet.");
      return;
    }

    cam_->setExposure(runtime_.exposure);
    XR_LOG_INFO("WebotsCamera: exposure=%.3f applied", runtime_.exposure);
  }

  void WarnUnsupportedGainIfNeeded() const
  {
    if (runtime_.gain == 0.0)
    {
      return;
    }

    XR_LOG_WARN(
        "WebotsCamera: gain=%.3f requested, but not supported by Webots Camera. "
        "Value recorded only.",
        runtime_.gain);
  }

  // ---- 仿真时间与节流 ----

  LibXR::MicrosecondTimestamp CurrentSimTimeUs() const
  {
    const double sim_time_s = robot_ != nullptr ? std::max(0.0, robot_->getTime()) : 0.0;
    return LibXR::MicrosecondTimestamp(
        static_cast<uint64_t>(std::llround(sim_time_s * 1000000.0)));
  }

  uint64_t CurrentSimStep() const
  {
    if (time_step_ms_ <= 0)
    {
      return 0;
    }

    const double sim_time_s = robot_ != nullptr ? std::max(0.0, robot_->getTime()) : 0.0;
    const uint64_t sim_time_ms = static_cast<uint64_t>(std::llround(sim_time_s * 1000.0));
    return sim_time_ms / static_cast<uint64_t>(time_step_ms_);
  }

  bool ConsumePublishBucket(uint64_t sim_step)
  {
    // 把时间轴切成固定发布桶，只要进入新的桶就发布一次。
    // 这样即使线程 sleep 存在抖动，发布节奏仍然由仿真时间决定。
    const uint64_t publish_bucket =
        sim_step / static_cast<uint64_t>(publish_interval_steps_);

    if (last_publish_bucket_ == UINT64_MAX)
    {
      last_publish_bucket_ = publish_bucket;
      return false;
    }
    if (publish_bucket == last_publish_bucket_)
    {
      return false;
    }

    last_publish_bucket_ = publish_bucket;
    return true;
  }

  // ---- Pose / Motion 采样 ----

  void RefreshCameraNode()
  {
    if (cam_node_ == nullptr && supervisor_ != nullptr)
    {
      cam_node_ = supervisor_->getFromDef(runtime_.pose_def_name.c_str());
    }
  }

  void PublishRotationTopic(const PoseSample& pose)
  {
    const LibXR::EulerAngle<float> eulr = pose.rotation.ToEulerAngle();
    XR_LOG_DEBUG("WebotsCamera: camera eulr: %.3f, %.3f, %.3f", eulr.Roll(),
                 eulr.Pitch(), eulr.Yaw());
    auto rotation = pose.rotation;
    gimbal_rotation_topic_.Publish(rotation);
  }

  static float WrapAngleDelta(float current, float previous)
  {
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;

    float delta = current - previous;
    while (delta > kPi)
    {
      delta -= kTwoPi;
    }
    while (delta < -kPi)
    {
      delta += kTwoPi;
    }
    return delta;
  }

  void ResetMotionWarmup(const PoseSample& pose)
  {
    last_pose_timestamp_us_ = static_cast<uint64_t>(pose.timestamp);
    last_pose_translation_ = pose.translation;
    last_pose_rotation_ = pose.rotation;
    last_linear_velocity_valid_ = false;
  }

  PoseSample ReadCameraPoseSample(LibXR::MicrosecondTimestamp timestamp)
  {
    PoseSample pose{};
    pose.timestamp = timestamp;

    RefreshCameraNode();
    if (cam_node_ == nullptr)
    {
      return pose;
    }

    // Webots Camera 的局部轴定义与项目里相机坐标系不同，这里先做一次
    // 固定补偿，再统一转成项目内使用的四元数表达。
    static const LibXR::RotationMatrix<double> compensation =
        LibXR::RotationMatrix<double>(0, 1, 0, 1, 0, 0, 0, 0, -1);

    const double* r9_cam = cam_node_->getOrientation();
    LibXR::RotationMatrix<double> r_final =
        LibXR::RotationMatrix<double>(r9_cam[0], r9_cam[1], r9_cam[2], r9_cam[3],
                                      r9_cam[4], r9_cam[5], r9_cam[6], r9_cam[7],
                                      r9_cam[8]) *
        compensation;

    const LibXR::Quaternion<double> quat = r_final;
    LibXR::Quaternion<float> quat_f(static_cast<float>(quat.x()), static_cast<float>(quat.y()),
                                    static_cast<float>(quat.z()), static_cast<float>(quat.w()));
    LibXR::EulerAngle<float> eulr = quat_f.ToEulerAngle();
    eulr = LibXR::EulerAngle<float>(-eulr.Pitch(), eulr.Yaw(), eulr.Roll());
    pose.rotation = eulr.ToQuaternion();

    const double* p3_cam = cam_node_->getPosition();
    pose.translation = LibXR::Position<float>(static_cast<float>(p3_cam[0]),
                                              static_cast<float>(p3_cam[1]),
                                              static_cast<float>(p3_cam[2]));
    return pose;
  }

  bool TryReadCameraMotionSample(const PoseSample& pose, MotionSample& motion)
  {
    const uint64_t current_timestamp_us = static_cast<uint64_t>(pose.timestamp);
    if (!last_pose_valid_)
    {
      last_pose_valid_ = true;
      ResetMotionWarmup(pose);
      return false;
    }

    if (current_timestamp_us <= last_pose_timestamp_us_)
    {
      ResetMotionWarmup(pose);
      return false;
    }

    const double dt =
        static_cast<double>(current_timestamp_us - last_pose_timestamp_us_) / 1000000.0;
    if (dt <= 0.0)
    {
      ResetMotionWarmup(pose);
      return false;
    }

    // Webots 这条链路里，pose 与图像是同一线程采样的，所以这里直接用连续
    // pose 差分估计角速度与线加速度，不再依赖节点原生 velocity 字段。
    const LibXR::EulerAngle<float> current_euler = pose.rotation.ToEulerAngle();
    const LibXR::EulerAngle<float> last_euler = last_pose_rotation_.ToEulerAngle();
    const float dt_f = static_cast<float>(dt);
    const LibXR::Position<float> current_linear_velocity(
        static_cast<float>((pose.translation.x() - last_pose_translation_.x()) / dt),
        static_cast<float>((pose.translation.y() - last_pose_translation_.y()) / dt),
        static_cast<float>((pose.translation.z() - last_pose_translation_.z()) / dt));

    motion.angular_velocity = LibXR::Position<float>(
        WrapAngleDelta(current_euler.Roll(), last_euler.Roll()) / dt_f,
        WrapAngleDelta(current_euler.Pitch(), last_euler.Pitch()) / dt_f,
        WrapAngleDelta(current_euler.Yaw(), last_euler.Yaw()) / dt_f);

    if (last_linear_velocity_valid_)
    {
      motion.linear_acceleration = LibXR::Position<float>(
          static_cast<float>((current_linear_velocity.x() - last_linear_velocity_.x()) / dt),
          static_cast<float>((current_linear_velocity.y() - last_linear_velocity_.y()) / dt),
          static_cast<float>((current_linear_velocity.z() - last_linear_velocity_.z()) / dt));
    }
    else
    {
      // 第一份有效速度还没有历史项可差分，线加速度先置零。
      motion.linear_acceleration = LibXR::Position<float>(0.0f, 0.0f, 0.0f);
    }

    last_pose_timestamp_us_ = current_timestamp_us;
    last_pose_translation_ = pose.translation;
    last_pose_rotation_ = pose.rotation;
    last_linear_velocity_ = current_linear_velocity;
    last_linear_velocity_valid_ = true;
    return true;
  }

  // ---- 帧写入与提交 ----

  bool WriteAndCommitImage(const unsigned char* rgba, LibXR::MicrosecondTimestamp timestamp)
  {
    if (!this->ImageSinkReady())
    {
      return false;
    }

    ImageFrame* image = this->GetWritableImage();
    if (image == nullptr)
    {
      fail_count_++;
      XR_LOG_WARN("WebotsCamera: writable image is null.");
      return false;
    }

    image->timestamp_us = static_cast<uint64_t>(timestamp);

    // 直接把 Webots 的 BGRA 图像转换后写入当前可写帧。
    cv::Mat src(frame_height, frame_width, CV_8UC4, const_cast<unsigned char*>(rgba));
    cv::Mat dst(frame_height, frame_width, CV_8UC3, image->data.data(), frame_step);
    cv::cvtColor(src, dst, cv::COLOR_BGRA2BGR);

    if (!this->CommitImage())
    {
      fail_count_++;
      XR_LOG_WARN("WebotsCamera: image commit failed.");
      return false;
    }

    fail_count_ = 0;
    return true;
  }

  // ---- 采集线程 ----

  static void ThreadFun(Self* self)
  {
    XR_LOG_INFO("Publishing image!");

    while (self->running_.load())
    {
      if (self->robot_ == nullptr || self->cam_ == nullptr)
      {
        break;
      }

      LibXR::Thread::Sleep(self->time_step_ms_);

      if (!self->ConsumePublishBucket(self->CurrentSimStep()))
      {
        continue;
      }
      if (!self->ImageSinkReady())
      {
        continue;
      }

      // 发布顺序固定为：仿真节流 -> pose -> motion -> image -> sink commit。
      const auto timestamp = self->CurrentSimTimeUs();
      auto pose = self->ReadCameraPoseSample(timestamp);
      self->PublishRotationTopic(pose);
      MotionSample motion{};
      if (!self->TryReadCameraMotionSample(pose, motion))
      {
        continue;
      }

      ImuStamped imu{
          .timestamp_us = static_cast<uint64_t>(timestamp),
          .rotation_wxyz = {
              pose.rotation.w(),
              pose.rotation.x(),
              pose.rotation.y(),
              pose.rotation.z(),
          },
          .translation_xyz = {
              pose.translation[0],
              pose.translation[1],
              pose.translation[2],
          },
          .angular_velocity_xyz = {
              motion.angular_velocity[0],
              motion.angular_velocity[1],
              motion.angular_velocity[2],
          },
          .linear_acceleration_xyz = {
              motion.linear_acceleration[0],
              motion.linear_acceleration[1],
              motion.linear_acceleration[2],
          },
      };
      self->PublishImu(imu);

      const unsigned char* rgba = self->cam_->getImage();
      if (rgba != nullptr)
      {
        self->WriteAndCommitImage(rgba, timestamp);
      }
      else
      {
        self->fail_count_++;
        XR_LOG_WARN("WebotsCamera: getImage returned null.");
      }

      if (self->fail_count_ > 5)
      {
        XR_LOG_ERROR("WebotsCamera failed repeatedly (%d)!", self->fail_count_);
      }
    }
  }

 private:
  RuntimeParam runtime_{};
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
  uint64_t last_publish_bucket_ = UINT64_MAX;
  bool last_pose_valid_ = false;
  uint64_t last_pose_timestamp_us_ = 0;
  LibXR::Position<float> last_pose_translation_{0.0f, 0.0f, 0.0f};
  LibXR::Quaternion<float> last_pose_rotation_{0.0f, 0.0f, 0.0f, 1.0f};
  bool last_linear_velocity_valid_ = false;
  LibXR::Position<float> last_linear_velocity_{0.0f, 0.0f, 0.0f};
};
