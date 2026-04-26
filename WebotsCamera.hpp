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

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
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
 * 3. 从 supervisor 读取相机 pose，并从 world 里的 IMU 设备读取真实运动量；
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

  struct ImuSensorNames
  {
    std::string gyro;
    std::string accelerometer;
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
    InitImuSensors();
    ValidateCameraGeometry();
    ApplyEnvOverridesOnStartup();
    ConfigureSamplingOnStartup();
    EnableImuSensors();
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
    DisableImuSensors();

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
      DisableImuSensors();
      cam_node_ = nullptr;
      pose_zero_calibrated_ = false;
      BindImuSensors();
      EnableImuSensors();
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

  static bool ReadEnvDouble(const char* name, double& value)
  {
    if (name == nullptr)
    {
      return false;
    }
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0')
    {
      return false;
    }

    char* end = nullptr;
    const double parsed = std::strtod(env, &end);
    if (end == env || !std::isfinite(parsed))
    {
      return false;
    }

    value = parsed;
    return true;
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

  std::string ResolveImuSensorPrefix() const
  {
    if (!runtime_.pose_def_name.empty())
    {
      return runtime_.pose_def_name;
    }

    const char* device_name = SafeCStr(runtime_.device_name);
    if (device_name[0] != '\0')
    {
      return std::string(device_name);
    }

    return "camera";
  }

  ImuSensorNames BuildImuSensorNames() const
  {
    const std::string prefix = ResolveImuSensorPrefix();
    return ImuSensorNames{
        .gyro = prefix + "_gyro",
        .accelerometer = prefix + "_accelerometer",
    };
  }

  void BindImuSensors()
  {
    imu_sensor_names_ = BuildImuSensorNames();
    gyro_ = robot_->getGyro(imu_sensor_names_.gyro);
    accelerometer_ = robot_->getAccelerometer(imu_sensor_names_.accelerometer);

    if (gyro_ == nullptr)
    {
      XR_LOG_ERROR("WebotsCamera: gyro '%s' not found in world.",
                   imu_sensor_names_.gyro.c_str());
      throw std::runtime_error("WebotsCamera: gyro device not found");
    }
    if (accelerometer_ == nullptr)
    {
      XR_LOG_ERROR("WebotsCamera: accelerometer '%s' not found in world.",
                   imu_sensor_names_.accelerometer.c_str());
      throw std::runtime_error("WebotsCamera: accelerometer device not found");
    }
  }

  void InitImuSensors()
  {
    BindImuSensors();
    XR_LOG_INFO("WebotsCamera: IMU devices gyro=%s accelerometer=%s",
                imu_sensor_names_.gyro.c_str(),
                imu_sensor_names_.accelerometer.c_str());
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

  void EnableImuSensors()
  {
    if (gyro_ != nullptr)
    {
      gyro_->enable(time_step_ms_);
    }
    if (accelerometer_ != nullptr)
    {
      accelerometer_->enable(time_step_ms_);
    }
  }

  void DisableImuSensors()
  {
    if (gyro_ != nullptr)
    {
      gyro_->disable();
      gyro_ = nullptr;
    }
    if (accelerometer_ != nullptr)
    {
      accelerometer_->disable();
      accelerometer_ = nullptr;
    }
  }

  void InitSupervisor(LibXR::HardwareContainer& hw)
  {
    supervisor_ = hw.FindOrExit<webots::Supervisor>({"supervisor"});
  }

  void ApplyEnvOverridesOnStartup()
  {
    double exposure = runtime_.exposure;
    if (ReadEnvDouble("XR_WEBOTS_CAMERA_EXPOSURE", exposure))
    {
      if (exposure >= 0.0)
      {
        XR_LOG_INFO("WebotsCamera: exposure env override %.3f -> %.3f",
                    runtime_.exposure, exposure);
        runtime_.exposure = exposure;
      }
      else
      {
        XR_LOG_WARN("WebotsCamera: ignored negative exposure override %.3f", exposure);
      }
    }
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

  PoseSample ReadCameraPoseSample(LibXR::MicrosecondTimestamp timestamp)
  {
    PoseSample pose{};
    pose.timestamp = timestamp;

    RefreshCameraNode();
    if (cam_node_ == nullptr)
    {
      return pose;
    }

    const double* r9_cam = cam_node_->getOrientation();
    const LibXR::RotationMatrix<double> camera_rotation_world(
        r9_cam[0], r9_cam[1], r9_cam[2], r9_cam[3], r9_cam[4], r9_cam[5],
        r9_cam[6], r9_cam[7], r9_cam[8]);

    static const LibXR::RotationMatrix<double> kNeutralGimbalToWorld(
        0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0,
        0.0, 0.0, 1.0);
    if (!pose_zero_calibrated_)
    {
      pose_zero_calibration_ =
          LibXR::RotationMatrix<double>(camera_rotation_world.transpose() *
                                        kNeutralGimbalToWorld);
      pose_zero_calibrated_ = true;
      XR_LOG_INFO("WebotsCamera captured SP-style pose zero calibration");
    }

    const LibXR::Quaternion<double> quat =
        camera_rotation_world * pose_zero_calibration_;
    pose.rotation = LibXR::Quaternion<float>(
        static_cast<float>(quat.w()), static_cast<float>(quat.x()),
        static_cast<float>(quat.y()), static_cast<float>(quat.z()));

    pose.translation = LibXR::Position<float>(0.0f, 0.0f, 0.0f);
    return pose;
  }

  LibXR::Position<float> RotateCameraNodeVectorToPublishedFrame(
      const double* raw_xyz) const
  {
    if (raw_xyz == nullptr)
    {
      return LibXR::Position<float>(0.0f, 0.0f, 0.0f);
    }

    const LibXR::Position<double> raw_vector(raw_xyz[0], raw_xyz[1], raw_xyz[2]);
    const LibXR::Position<double> published_vector =
        pose_zero_calibration_.transpose() * raw_vector;
    return LibXR::Position<float>(static_cast<float>(published_vector.x()),
                                  static_cast<float>(published_vector.y()),
                                  static_cast<float>(published_vector.z()));
  }

  bool ReadImuMotionSample(MotionSample& motion) const
  {
    if (gyro_ == nullptr || accelerometer_ == nullptr || !pose_zero_calibrated_)
    {
      return false;
    }

    const double* angular_velocity = gyro_->getValues();
    const double* linear_acceleration = accelerometer_->getValues();
    if (angular_velocity == nullptr || linear_acceleration == nullptr)
    {
      return false;
    }

    // Webots 传感器原始值位于 camera node 局部轴；这里统一旋到当前发布的
    // SP/BMI088 风格姿态帧，确保 rotation / gyro / accel 三者坐标语义一致。
    motion.angular_velocity =
        RotateCameraNodeVectorToPublishedFrame(angular_velocity);
    motion.linear_acceleration =
        RotateCameraNodeVectorToPublishedFrame(linear_acceleration);
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

      // 发布顺序固定为：仿真节流 -> pose -> imu -> image -> sink commit。
      const auto timestamp = self->CurrentSimTimeUs();
      auto pose = self->ReadCameraPoseSample(timestamp);
      self->PublishRotationTopic(pose);
      MotionSample motion{};
      if (!self->ReadImuMotionSample(motion))
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
  LibXR::Topic::Domain gimbal_domain_ = LibXR::Topic::Domain("gimbal");
  LibXR::Topic gimbal_rotation_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation", &gimbal_domain_);

  webots::Robot* robot_{};
  webots::Camera* cam_ = nullptr;
  webots::Gyro* gyro_ = nullptr;
  webots::Accelerometer* accelerometer_ = nullptr;
  webots::Node* cam_node_ = nullptr;
  webots::Supervisor* supervisor_ = nullptr;
  ImuSensorNames imu_sensor_names_{};
  int time_step_ms_ = 0;
  int sample_period_ms_ = 33;
  int publish_interval_steps_ = 1;

  std::atomic<bool> running_{false};
  LibXR::Thread capture_thread_{};
  int fail_count_ = 0;
  uint64_t last_publish_bucket_ = UINT64_MAX;
  bool pose_zero_calibrated_ = false;
  LibXR::RotationMatrix<double> pose_zero_calibration_{};
};
