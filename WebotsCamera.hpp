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
#include <limits>
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
 * 2. 每个 world step 发布一组原始 imu（gyro/accl/quat）；
 * 3. 按基础分频发布图像，并响应一次性 `sensor_sync_cmd` 探针；
 * 4. 从 supervisor 读取相机 pose，把图像写入 `CameraBase::ImageFrame`
 *    并交给已注册的图像 sink。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class WebotsCamera : public LibXR::Application, public CameraBase<CameraInfoV>
{
 public:
  using Self = WebotsCamera<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using ImageFrame = typename Base::ImageFrame;
  using GyroStamped = typename Base::GyroStamped;
  using AcclStamped = typename Base::AcclStamped;
  using QuatStamped = typename Base::QuatStamped;
  using ImageEvent = typename Base::ImageEvent;
  using SensorSyncCmd = typename Base::SensorSyncCmd;

  static inline constexpr CameraInfo camera_info = Base::camera_info;
  static constexpr const char* kDefaultTopicPrefix = "camera";
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
    // 当前发布时刻的相机姿态。rotation 使用对外发布的 SP/BMI088 风格帧。
    LibXR::MicrosecondTimestamp timestamp{};
    LibXR::Quaternion<float> rotation{};
    LibXR::Position<float> translation{};
  };

  struct MotionSample
  {
    // 与 PoseSample 同帧下的运动量，坐标语义必须和 rotation_wxyz 一致。
    LibXR::Position<float> angular_velocity{};
    LibXR::Position<float> linear_acceleration{};
  };

  struct ImuSensorNames
  {
    // world 里通过统一前缀拼接出来的设备名，例如 camera_gyro。
    std::string gyro;
    std::string accelerometer;
  };

  struct SimClockSample
  {
    // 这里的 step / timestamp 都来自同一次 Webots 仿真时钟采样，避免跨调用抖动。
    uint64_t step = 0;
    LibXR::MicrosecondTimestamp timestamp{};
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
        device_topic_prefix_((runtime.device_name != nullptr && runtime.device_name[0] != '\0')
                                 ? runtime.device_name
                                 : kDefaultTopicPrefix),
        gyro_topic_name_(device_topic_prefix_ + "_gyro"),
        accl_topic_name_(device_topic_prefix_ + "_accl"),
        quat_topic_name_(device_topic_prefix_ + "_quat"),
        image_event_topic_name_(device_topic_prefix_ + "_image_event"),
        sensor_sync_cmd_topic_name_("sensor_sync_cmd"),
        raw_gyro_topic_(LibXR::Topic::FindOrCreate<GyroStamped>(gyro_topic_name_.c_str())),
        raw_accl_topic_(LibXR::Topic::FindOrCreate<AcclStamped>(accl_topic_name_.c_str())),
        raw_quat_topic_(LibXR::Topic::FindOrCreate<QuatStamped>(quat_topic_name_.c_str())),
        image_event_topic_(
            LibXR::Topic::FindOrCreate<ImageEvent>(image_event_topic_name_.c_str())),
        sensor_sync_cmd_topic_(
            LibXR::Topic::FindOrCreate<SensorSyncCmd>(sensor_sync_cmd_topic_name_.c_str())),
        sensor_sync_cmd_sub_(sensor_sync_cmd_topic_),
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
    sensor_sync_cmd_sub_.StartWaiting();
    StartCaptureThread();

    XR_LOG_PASS(
        "Webots camera enabled: name=%s, capture_period=%d ms, world_dt=%d ms, image_divisor=%d",
        runtime_.device_name, camera_sample_period_ms_, time_step_ms_, base_image_interval_steps_);

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
    // 优先用 pose_def_name，对齐 supervisor 查询与 IMU 设备命名。
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
    // 约定 world 里的 IMU 设备名为 <prefix>_gyro / <prefix>_accelerometer。
    const std::string prefix = ResolveImuSensorPrefix();
    return ImuSensorNames{
        .gyro = prefix + "_gyro",
        .accelerometer = prefix + "_accelerometer",
    };
  }

  void BindImuSensors()
  {
    // 这里只做设备绑定，不做 enable，便于运行时在 pose_def 切换后重绑。
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
    // Webots Camera 的实际输出几何必须和模板参数完全一致，否则下游 frame
    // 视图与步长都会错。
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
    // Camera 每个 world step 都使能，真正的发图频率由本地图像调度器控制。
    camera_sample_period_ms_ = time_step_ms_;
    base_image_interval_steps_ =
        ComputePublishIntervalSteps(FpsToPeriodMs(runtime_.fps, time_step_ms_),
                                    time_step_ms_);
    ResetImageSchedule();
    last_processed_step_ = std::numeric_limits<uint64_t>::max();
    cam_->enable(camera_sample_period_ms_);
  }

  void EnableImuSensors()
  {
    // IMU 直接按 world basicTimeStep 采样，避免对角速度/加速度再做插值。
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
    // disable 后把指针清空，避免后续误以为设备仍可读。
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
    // 目前仅保留曝光 env 覆盖，方便做 detector/tracker A/B。
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
    // 采集线程把 pose / imu / image 串成同一条发布时间线。
    running_.store(true);
    capture_thread_.Create(this, CaptureThreadMain, "WebotsCameraThread",
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
    if (camera_sample_period_ms_ != time_step_ms_)
    {
      camera_sample_period_ms_ = time_step_ms_;
      cam_->disable();
      cam_->enable(camera_sample_period_ms_);
    }

    base_image_interval_steps_ =
        ComputePublishIntervalSteps(FpsToPeriodMs(runtime_.fps, time_step_ms_),
                                    time_step_ms_);
    ResetImageSchedule();
    last_processed_step_ = std::numeric_limits<uint64_t>::max();
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

  SimClockSample ReadSimClockSample() const
  {
    const double sim_time_s = robot_ != nullptr ? std::max(0.0, robot_->getTime()) : 0.0;
    SimClockSample clock{};
    clock.timestamp = LibXR::MicrosecondTimestamp(
        static_cast<uint64_t>(std::llround(sim_time_s * 1000000.0)));
    if (time_step_ms_ <= 0)
    {
      return clock;
    }

    const uint64_t sim_time_ms = static_cast<uint64_t>(std::llround(sim_time_s * 1000.0));
    clock.step = sim_time_ms / static_cast<uint64_t>(time_step_ms_);
    return clock;
  }

  bool EnterNewStep(uint64_t sim_step)
  {
    if (last_processed_step_ == sim_step)
    {
      return false;
    }

    last_processed_step_ = sim_step;
    return true;
  }

  void ResetImageSchedule()
  {
    // 下一次重新初始化后，首张图会以“当前 step”为基准重新建立节拍。
    image_schedule_initialized_ = false;
    next_image_step_ = 0;
    next_image_interval_stretched_ = false;
  }

  void InitializeImageScheduleIfNeeded(uint64_t sim_step)
  {
    if (image_schedule_initialized_)
    {
      return;
    }

    // 调度器复位后的第一张图，以当前 step 作为新的节拍零点。
    image_schedule_initialized_ = true;
    next_image_step_ = sim_step;
  }

  bool ShouldPublishImageAtStep(uint64_t sim_step) const
  {
    return sim_step >= next_image_step_;
  }

  void StretchNextImageIntervalForProbe()
  {
    // 一次性探针的下位机语义只有一个动作：
    // 把“下一次”图像发布时间再向后推一个基础周期，于是当前间隔从 N 变成 2N。
    if (!image_schedule_initialized_ || next_image_interval_stretched_)
    {
      return;
    }

    next_image_step_ += static_cast<uint64_t>(base_image_interval_steps_);
    next_image_interval_stretched_ = true;
  }

  void AdvanceImageScheduleAfterPublish()
  {
    // 探针只影响一个图像间隔；当前图发布后立即恢复基础节拍。
    next_image_step_ += static_cast<uint64_t>(base_image_interval_steps_);
    next_image_interval_stretched_ = false;
  }

  // ---- Pose / Motion 采样 ----

  void RefreshCameraNode()
  {
    // camera node 通过 supervisor 按 DEF 延迟绑定，允许 world 初始化稍晚完成。
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
    // 这个 topic 只发布已经过零位标定后的外部语义姿态，不暴露原始 camera node 姿态。
    auto rotation = pose.rotation;
    gimbal_rotation_topic_.Publish(rotation);
  }

  void PollSensorSyncProbeCommand()
  {
    if (!sensor_sync_cmd_sub_.Available())
    {
      return;
    }

    const SensorSyncCmd cmd = sensor_sync_cmd_sub_.GetData();
    sensor_sync_cmd_sub_.StartWaiting();
    (void)cmd;

    if (next_image_interval_stretched_)
    {
      XR_LOG_WARN("WebotsCamera: ignored duplicated sensor_sync_cmd while probe is armed");
      return;
    }

    StretchNextImageIntervalForProbe();
    XR_LOG_INFO("WebotsCamera: accepted sensor_sync_cmd, next image interval stretched");
  }

  void PublishRawImu(const PoseSample& pose, const MotionSample& motion,
                     LibXR::MicrosecondTimestamp timestamp)
  {
    GyroStamped gyro{
        .sensor_timestamp_us = static_cast<uint64_t>(timestamp),
        .angular_velocity_xyz = {
            motion.angular_velocity[0],
            motion.angular_velocity[1],
            motion.angular_velocity[2],
        },
    };
    AcclStamped accl{
        .sensor_timestamp_us = static_cast<uint64_t>(timestamp),
        .linear_acceleration_xyz = {
            motion.linear_acceleration[0],
            motion.linear_acceleration[1],
            motion.linear_acceleration[2],
        },
    };
    QuatStamped quat{
        .sensor_timestamp_us = static_cast<uint64_t>(timestamp),
        .rotation_wxyz = {
            pose.rotation.w(),
            pose.rotation.x(),
            pose.rotation.y(),
            pose.rotation.z(),
        },
    };

    raw_gyro_topic_.Publish(gyro);
    raw_accl_topic_.Publish(accl);
    raw_quat_topic_.Publish(quat);
  }

  void PublishImageEvent(LibXR::MicrosecondTimestamp timestamp, uint64_t sim_step)
  {
    // image_event 是主机侧同步基线。即使后面的图像 payload 被丢弃，这条事件也必须先发。
    ImageEvent event{
        .sensor_timestamp_us = static_cast<uint64_t>(timestamp),
        .sensor_step_index = static_cast<uint32_t>(sim_step),
    };
    image_event_topic_.Publish(event);
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

    // 目标发布帧采用当前系统已经接受的 SP/BMI088 风格中立姿态语义：
    // x 向右、y 向前、z 向上。第一次读到相机姿态时冻结一份零位标定矩阵，
    // 后续所有 rotation / gyro / accel 都以这份标定为准。
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

    // 当前 WebotsCamera 只负责旋转语义；平移仍维持历史上的零平移发布策略。
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

    const Eigen::Matrix<double, 3, 1> raw_vector(raw_xyz[0], raw_xyz[1], raw_xyz[2]);
    // pose_zero_calibration_ 是 “camera_node -> published frame” 的零位修正。
    // 对向量做 transpose 等价于把 raw camera-node 向量旋到当前对外发布帧。
    const Eigen::Matrix<double, 3, 1> published_vector =
        pose_zero_calibration_.transpose() * raw_vector;
    return LibXR::Position<float>(static_cast<float>(published_vector.x()),
                                  static_cast<float>(published_vector.y()),
                                  static_cast<float>(published_vector.z()));
  }

  bool ReadImuMotionSample(MotionSample& motion) const
  {
    // 在零位标定完成前，不发布 IMU，避免 rotation 与 gyro/accel 语义混帧。
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
      consecutive_fail_count_++;
      XR_LOG_WARN("WebotsCamera: writable image is null.");
      return false;
    }

    image->timestamp_us = static_cast<uint64_t>(timestamp);

    // image_event 已经先发，这里只负责图像 payload 本身；即使 sink 提交失败，
    // 主机侧仍然能保留同步基线事件。
    cv::Mat src(frame_height, frame_width, CV_8UC4, const_cast<unsigned char*>(rgba));
    cv::Mat dst(frame_height, frame_width, CV_8UC3, image->data.data(), frame_step);
    cv::cvtColor(src, dst, cv::COLOR_BGRA2BGR);

    if (!this->CommitImage())
    {
      consecutive_fail_count_++;
      XR_LOG_WARN("WebotsCamera: image commit failed.");
      return false;
    }

    consecutive_fail_count_ = 0;
    return true;
  }

  // ---- 采集线程 ----

  static void CaptureThreadMain(Self* self)
  {
    XR_LOG_INFO("Publishing image!");

    while (self->running_.load())
    {
      if (self->robot_ == nullptr || self->cam_ == nullptr)
      {
        break;
      }

      // Webots 控制器线程不直接阻塞在 robot->step，这里按 basicTimeStep 节拍轮询。
      LibXR::Thread::Sleep(self->time_step_ms_);

      const SimClockSample clock = self->ReadSimClockSample();
      if (!self->EnterNewStep(clock.step))
      {
        continue;
      }

      self->InitializeImageScheduleIfNeeded(clock.step);
      self->PollSensorSyncProbeCommand();

      auto pose = self->ReadCameraPoseSample(clock.timestamp);
      self->PublishRotationTopic(pose);
      MotionSample motion{};
      if (!self->ReadImuMotionSample(motion))
      {
        continue;
      }

      self->PublishRawImu(pose, motion, clock.timestamp);

      if (!self->ShouldPublishImageAtStep(clock.step))
      {
        continue;
      }

      self->AdvanceImageScheduleAfterPublish();
      self->PublishImageEvent(clock.timestamp, clock.step);

      const unsigned char* rgba = self->cam_->getImage();
      if (rgba != nullptr)
      {
        (void)self->WriteAndCommitImage(rgba, clock.timestamp);
      }
      else
      {
        self->consecutive_fail_count_++;
        XR_LOG_WARN("WebotsCamera: getImage returned null.");
      }

      if (self->consecutive_fail_count_ > 5)
      {
        XR_LOG_ERROR("WebotsCamera failed repeatedly (%d)!",
                     self->consecutive_fail_count_);
      }
    }
  }

 private:
  RuntimeParam runtime_{};
  std::string device_topic_prefix_{};
  std::string gyro_topic_name_{};
  std::string accl_topic_name_{};
  std::string quat_topic_name_{};
  std::string image_event_topic_name_{};
  std::string sensor_sync_cmd_topic_name_{};
  // rotation topic 保持在 gimbal domain，沿用历史消费者的查找路径。
  LibXR::Topic::Domain gimbal_domain_ = LibXR::Topic::Domain("gimbal");
  LibXR::Topic gimbal_rotation_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation", &gimbal_domain_);
  LibXR::Topic raw_gyro_topic_{};
  LibXR::Topic raw_accl_topic_{};
  LibXR::Topic raw_quat_topic_{};
  LibXR::Topic image_event_topic_{};
  LibXR::Topic sensor_sync_cmd_topic_{};
  LibXR::Topic::ASyncSubscriber<SensorSyncCmd> sensor_sync_cmd_sub_;

  webots::Robot* robot_{};
  webots::Camera* cam_ = nullptr;
  webots::Gyro* gyro_ = nullptr;
  webots::Accelerometer* accelerometer_ = nullptr;
  webots::Node* cam_node_ = nullptr;
  webots::Supervisor* supervisor_ = nullptr;
  ImuSensorNames imu_sensor_names_{};
  int time_step_ms_ = 0;
  int camera_sample_period_ms_ = 33;
  int base_image_interval_steps_ = 1;

  std::atomic<bool> running_{false};
  LibXR::Thread capture_thread_{};
  int consecutive_fail_count_ = 0;
  uint64_t last_processed_step_ = std::numeric_limits<uint64_t>::max();
  uint64_t next_image_step_{0};
  bool next_image_interval_stretched_{false};
  bool image_schedule_initialized_{false};
  bool pose_zero_calibrated_ = false;
  // 首帧冻结的零位标定矩阵：把 raw camera node 姿态/向量对齐到外部发布帧。
  LibXR::RotationMatrix<double> pose_zero_calibration_{};
};
