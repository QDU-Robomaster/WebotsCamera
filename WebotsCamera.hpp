#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Webots 相机与 IMU 采集端
constructor_args:
  - runtime:
      device_name: "camera"
      fps: 30
      exposure: 1.0
      gain: 0.0
      pose_def_name: "camera"
      raw_topic_domain_name: "mcu"
      trigger_gpio_name: "CAMERA"
      trigger_active_level: true
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
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string_view>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "CameraBase.hpp"
#include "app_framework.hpp"
#include "gpio.hpp"
#include "libxr.hpp"
#include "libxr_system.hpp"
#include "libxr_string.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "transform.hpp"

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Robot.hpp>

extern webots::Robot* _libxr_webots_robot_handle;

/**
 * @class WebotsCamera
 * @brief Webots 仿真环境中的相机与 IMU 数据源。
 *
 * @tparam CameraInfoV 编译期相机模型，必须是紧密排列的 BGR8 图像。
 *
 * @details
 * 模块按 Webots step 读取 `Gyro`、`Accelerometer` 和 `InertialUnit`，并把
 * 原始 IMU 样本发布到 `<device_name>_gyro`、`<device_name>_accl`、
 * `<device_name>_quat`。图像不再由本模块按 topic 命令调度，而是把自身注册为
 * `LibXR::GPIO`，由 CameraSync 像真实 MCU 一样翻转触发线；进入有效电平的边沿
 * 才提交一帧图像。
 *
 * WebotsCamera 只模拟传感器端点。同步命令、分频拉长、seq 回执和 Host/MCU
 * SharedTopic 边界全部由 CameraSync / CameraFrameSync 负责。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class WebotsCamera : public LibXR::Application,
                     public CameraBase<CameraInfoV>,
                     public LibXR::GPIO
{
 public:
  using Self = WebotsCamera<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using ImageFrame = typename Base::ImageFrame;
  using ImuSample = Eigen::Matrix<float, 3, 1>;
  using QuatSample = LibXR::Quaternion<float>;

  static inline constexpr CameraInfo camera_info = Base::camera_info;
  static constexpr int channel_count = 3;
  static constexpr int frame_width = static_cast<int>(camera_info.width);
  static constexpr int frame_height = static_cast<int>(camera_info.height);
  static constexpr std::size_t frame_step = static_cast<std::size_t>(camera_info.step);

  static_assert(camera_info.encoding == CameraTypes::Encoding::BGR8,
                "WebotsCamera requires BGR8 output encoding");
  static_assert(frame_step == static_cast<std::size_t>(camera_info.width) * channel_count,
                "WebotsCamera requires packed BGR step");

  /**
   * @struct PoseSample
   * @brief 单个 Webots step 内的姿态样本。
   */
  struct PoseSample
  {
    LibXR::Quaternion<float> rotation{};  ///< 发布坐标系相对 world 的姿态，wxyz。
    LibXR::Position<float> translation{};  ///< 相机平移，当前 Webots 端固定为 0。
  };

  /**
   * @struct MotionSample
   * @brief 单个 Webots step 内的运动样本。
   */
  struct MotionSample
  {
    LibXR::Position<float> angular_velocity{};  ///< 角速度，单位 rad/s。
    LibXR::Position<float> linear_acceleration{};  ///< 线加速度，单位 m/s^2。
  };

  /**
   * @struct RuntimeParam
   * @brief xrobot YAML 传入的运行时参数。
   */
  struct RuntimeParam
  {
    /// Webots Camera 设备名，也是原始 IMU topic 前缀。
    std::string_view device_name = "camera";

    /// 图像目标频率，实际周期量化到 Webots step。
    int fps = 30;

    /// Webots Camera 曝光值。
    double exposure = 1.0;

    /// 保留参数；Webots Camera 不支持 gain，非零值会被忽略。
    double gain = 0.0;

    /// IMU 设备名前缀。
    std::string_view pose_def_name = "camera";

    /// 原始图像 topic 名。
    std::string_view image_topic_name = "camera_image";

    /// 同步后 IMU topic 名。
    std::string_view imu_topic_name = "camera_imu";

    /// 原始 IMU topic domain。实机路径通常是 MCU 域，仿真 detector-only 可改成默认域。
    std::string_view raw_topic_domain_name = "mcu";

    /// 注册到 HardwareContainer 的相机触发 GPIO 名称，必须与 CameraSync 参数一致。
    std::string_view trigger_gpio_name = "CAMERA";

    /// 进入该电平的边沿提交图像；true 表示高有效，false 表示低有效。
    bool trigger_active_level = true;
  };

  /**
   * @brief 构造并启动 Webots 相机采集端。
   * @param hw libxr 硬件容器，当前模块不额外挂载硬件对象。
   * @param app 应用管理器。
   * @param runtime 运行时参数。
   */
  explicit WebotsCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                        RuntimeParam runtime)
      : Base(hw, runtime.device_name, runtime.image_topic_name, runtime.imu_topic_name),
        target_fps_(runtime.fps),
        exposure_(runtime.exposure),
        gain_(runtime.gain),
        gyro_device_name_(runtime.pose_def_name, "_gyro"),
        accl_device_name_(runtime.pose_def_name, "_accelerometer"),
        quat_device_name_(runtime.pose_def_name, "_inertial_unit"),
        raw_topic_domain_name_(runtime.raw_topic_domain_name),
        raw_topic_domain_(raw_topic_domain_name_.CStr()),
        gyro_topic_name_(runtime.device_name, "_gyro"),
        accl_topic_name_(runtime.device_name, "_accl"),
        quat_topic_name_(runtime.device_name, "_quat"),
        trigger_gpio_name_(runtime.trigger_gpio_name),
        raw_gyro_topic_(LibXR::Topic::FindOrCreate<ImuSample>(
            gyro_topic_name_.CStr(), &raw_topic_domain_)),
        raw_accl_topic_(LibXR::Topic::FindOrCreate<ImuSample>(
            accl_topic_name_.CStr(), &raw_topic_domain_)),
        raw_quat_topic_(LibXR::Topic::FindOrCreate<QuatSample>(
            quat_topic_name_.CStr(), &raw_topic_domain_)),
        robot_(_libxr_webots_robot_handle),
        trigger_active_level_(runtime.trigger_active_level)
  {
    XR_LOG_INFO("Starting WebotsCamera!");

    if (target_fps_ <= 0)
    {
      XR_LOG_ERROR("WebotsCamera: runtime.fps must be positive, got %d", target_fps_);
      throw std::invalid_argument("WebotsCamera: runtime.fps must be positive");
    }
    if (runtime.device_name.empty() || runtime.pose_def_name.empty() ||
        runtime.image_topic_name.empty() || runtime.imu_topic_name.empty())
    {
      XR_LOG_ERROR("WebotsCamera: runtime names must not be empty");
      throw std::invalid_argument("WebotsCamera: required runtime string is empty");
    }
    if (runtime.raw_topic_domain_name.empty() || runtime.trigger_gpio_name.empty())
    {
      XR_LOG_ERROR("WebotsCamera: runtime sync names must not be empty");
      throw std::invalid_argument("WebotsCamera: sync runtime string is empty");
    }

    hw.Register(LibXR::Entry<LibXR::GPIO>{*this, {trigger_gpio_name_.CStr()}});

    InitRobot();
    InitCamera();
    InitImuSensors();
    ValidateCameraGeometry();
    ConfigureSamplingOnStartup();
    ApplyExposure();
    IgnoreUnsupportedGainRequest();
    StartCaptureThread();

    XR_LOG_PASS(
        "Webots camera enabled: name=%s, capture_period=%d ms, world_dt=%d ms, image_divisor=%d",
        this->Name(), base_image_interval_steps_ * time_step_ms_, time_step_ms_,
        base_image_interval_steps_);

    app.Register(*this);
  }

  /** @brief 请求采集线程退出。 */
  ~WebotsCamera() { running_.store(false); }

  /** @brief 当前模块无周期监控输出。 */
  void OnMonitor() override {}

  /** @brief 读取当前仿真触发线电平。 */
  bool Read() override { return trigger_level_.load(std::memory_order_relaxed); }

  /**
   * @brief 写入仿真触发线。
   *
   * CameraSync 在 MCU 侧 IMU 回调里翻转该 GPIO。这里仅在进入有效电平的边沿提交
   * 图像，和真实相机触发脚行为一致；同步命令/回执不在本模块处理。
   */
  void Write(bool value) override
  {
    const bool old = trigger_level_.exchange(value, std::memory_order_relaxed);
    if (old == value || value != trigger_active_level_)
    {
      return;
    }

    const LibXR::MicrosecondTimestamp timestamp = LibXR::Timebase::GetMicroseconds();
    CommitImageSample(timestamp);
    ReportRepeatedFailureIfNeeded();
  }

  /** @brief Webots 仿真 GPIO 不需要真实中断使能。 */
  LibXR::ErrorCode EnableInterrupt() override { return LibXR::ErrorCode::OK; }

  /** @brief Webots 仿真 GPIO 不需要真实中断禁用。 */
  LibXR::ErrorCode DisableInterrupt() override { return LibXR::ErrorCode::OK; }

  /** @brief 保存 GPIO 配置，便于调试时确认 CameraSync 配置过该触发脚。 */
  LibXR::ErrorCode SetConfig(Configuration config) override
  {
    trigger_gpio_config_ = config;
    return LibXR::ErrorCode::OK;
  }

  /** @brief 更新 Webots Camera 曝光值。 */
  void SetExposure(double exposure) override
  {
    exposure_ = exposure;
    ApplyExposure();
  }

  /** @brief Webots Camera 不支持 gain；非零值只记录 warning 并清零。 */
  void SetGain(double gain) override
  {
    gain_ = gain;
    IgnoreUnsupportedGainRequest();
  }

 private:
  // ---- 基础工具 ----

  static int FpsToPeriodMs(int fps)
  {
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

    time_step_ms_ = static_cast<int>(std::lround(robot_->getBasicTimeStep()));
    if (time_step_ms_ <= 0)
    {
      XR_LOG_ERROR("Webots basic timestep is invalid: %d ms", time_step_ms_);
      throw std::runtime_error("WebotsCamera: invalid basic timestep");
    }
  }

  void InitCamera()
  {
    cam_ = robot_->getCamera(this->Name());
    if (cam_ == nullptr)
    {
      XR_LOG_ERROR("Webots Camera '%s' not found!", this->Name());
      throw std::runtime_error("WebotsCamera: camera device not found");
    }
  }

  void BindImuSensors()
  {
    // 设备名由 pose_def_name 派生，world 中必须存在同名节点。
    gyro_ = robot_->getGyro(gyro_device_name_.CStr());
    accelerometer_ = robot_->getAccelerometer(accl_device_name_.CStr());
    inertial_unit_ = robot_->getInertialUnit(quat_device_name_.CStr());

    if (gyro_ == nullptr)
    {
      XR_LOG_ERROR("WebotsCamera: gyro '%s' not found in world.",
                   gyro_device_name_.CStr());
      throw std::runtime_error("WebotsCamera: gyro device not found");
    }
    if (accelerometer_ == nullptr)
    {
      XR_LOG_ERROR("WebotsCamera: accelerometer '%s' not found in world.",
                   accl_device_name_.CStr());
      throw std::runtime_error("WebotsCamera: accelerometer device not found");
    }
    if (inertial_unit_ == nullptr)
    {
      XR_LOG_ERROR("WebotsCamera: inertial unit '%s' not found in world.",
                   quat_device_name_.CStr());
      throw std::runtime_error("WebotsCamera: inertial unit device not found");
    }
  }

  void InitImuSensors()
  {
    BindImuSensors();
    if (gyro_ != nullptr)
    {
      gyro_->enable(time_step_ms_);
    }
    if (accelerometer_ != nullptr)
    {
      accelerometer_->enable(time_step_ms_);
    }
    if (inertial_unit_ != nullptr)
    {
      inertial_unit_->enable(time_step_ms_);
    }
    XR_LOG_INFO("WebotsCamera: IMU devices gyro=%s accelerometer=%s inertial_unit=%s",
                gyro_device_name_.CStr(), accl_device_name_.CStr(),
                quat_device_name_.CStr());
  }

  void ValidateCameraGeometry() const
  {
    // CameraBase 的帧视图依赖编译期几何，启动阶段必须校验 Webots 实际输出。
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
    // Camera enable 周期只决定 Webots 渲染负载；真正提交图像由 GPIO 触发边沿决定。
    base_image_interval_steps_ = ComputePublishIntervalSteps(FpsToPeriodMs(target_fps_),
                                                             time_step_ms_);
    const int camera_sampling_period_ms = base_image_interval_steps_ * time_step_ms_;
    last_processed_step_ = std::numeric_limits<uint64_t>::max();
    cam_->enable(camera_sampling_period_ms);
  }

  void StartCaptureThread()
  {
    // REALTIME 线程会被 libxr Webots timebase 纳入 step 推进屏障。
    running_.store(true);
    capture_thread_.Create(this, CaptureThreadMain, "WebotsCameraThread",
                           static_cast<size_t>(1024 * 128),
                           LibXR::Thread::Priority::REALTIME);
  }

  void ApplyExposure()
  {
    if (cam_ == nullptr)
    {
      XR_LOG_WARN("WebotsCamera: camera not ready yet.");
      return;
    }

    cam_->setExposure(exposure_);
    XR_LOG_INFO("WebotsCamera: exposure=%.3f applied", exposure_);
  }

  void IgnoreUnsupportedGainRequest()
  {
    if (gain_ == 0.0)
    {
      return;
    }

    XR_LOG_WARN(
        "WebotsCamera: gain=%.3f requested, but Webots Camera does not support gain. "
        "Request ignored.",
        gain_);
    gain_ = 0.0;
  }

  // ---- 仿真时间与节流 ----

  bool EnterNewStep(uint64_t sim_step)
  {
    if (last_processed_step_ == sim_step)
    {
      return false;
    }

    last_processed_step_ = sim_step;
    return true;
  }

  // ---- Pose / Motion 采样 ----

  void PublishGimbalRotation(const PoseSample& pose)
  {
#if LIBXR_LOG_LEVEL >= 4
    const LibXR::EulerAngle<float> eulr = pose.rotation.ToEulerAngle();
    XR_LOG_DEBUG("WebotsCamera: camera euler roll_x=%.3f pitch_y=%.3f yaw_z=%.3f",
                 eulr.Roll(), eulr.Pitch(), eulr.Yaw());
#endif
    // 沿用 gimbal 域历史 topic；数据与 camera_quat 同源。
    auto rotation = pose.rotation;
    gimbal_rotation_topic_.Publish(rotation);
  }

  void PublishRawImu(const PoseSample& pose, const MotionSample& motion,
                     LibXR::MicrosecondTimestamp timestamp)
  {
    ImuSample gyro(motion.angular_velocity[0], motion.angular_velocity[1],
                   motion.angular_velocity[2]);
    ImuSample accl(motion.linear_acceleration[0], motion.linear_acceleration[1],
                   motion.linear_acceleration[2]);
    QuatSample quat(pose.rotation.w(), pose.rotation.x(), pose.rotation.y(),
                    pose.rotation.z());

    raw_gyro_topic_.Publish(gyro, timestamp);
    raw_accl_topic_.Publish(accl, timestamp);
    raw_quat_topic_.Publish(quat, timestamp);
  }

  bool ReadCameraPoseSample(PoseSample& pose)
  {
    if (inertial_unit_ == nullptr)
    {
      return false;
    }

    const double* raw_xyzw = inertial_unit_->getQuaternion();
    if (raw_xyzw == nullptr)
    {
      return false;
    }

    // Webots 返回 xyzw；CameraBase/Topic 侧统一发布 wxyz。
    pose.rotation = LibXR::Quaternion<float>(
        static_cast<float>(raw_xyzw[3]), static_cast<float>(raw_xyzw[0]),
        static_cast<float>(raw_xyzw[1]), static_cast<float>(raw_xyzw[2]));

    // WebotsCamera 只发布姿态，平移仍由下游静态外参处理。
    pose.translation = LibXR::Position<float>(0.0f, 0.0f, 0.0f);
    return true;
  }

  bool ReadImuMotionSample(MotionSample& motion) const
  {
    if (gyro_ == nullptr || accelerometer_ == nullptr)
    {
      return false;
    }

    const double* angular_velocity = gyro_->getValues();
    const double* linear_acceleration = accelerometer_->getValues();
    if (angular_velocity == nullptr || linear_acceleration == nullptr)
    {
      return false;
    }

    // 坐标轴由 world 中传感器节点的安装方向保证，这里不做零位补偿。
    motion.angular_velocity =
        LibXR::Position<float>(static_cast<float>(angular_velocity[0]),
                               static_cast<float>(angular_velocity[1]),
                               static_cast<float>(angular_velocity[2]));
    motion.linear_acceleration =
        LibXR::Position<float>(static_cast<float>(linear_acceleration[0]),
                               static_cast<float>(linear_acceleration[1]),
                               static_cast<float>(linear_acceleration[2]));
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

  void CommitImageSample(LibXR::MicrosecondTimestamp timestamp)
  {
    const unsigned char* rgba = cam_->getImage();
    if (rgba != nullptr)
    {
      (void)WriteAndCommitImage(rgba, timestamp);
      return;
    }

    consecutive_fail_count_++;
    XR_LOG_WARN("WebotsCamera: getImage returned null.");
  }

  void ReportRepeatedFailureIfNeeded() const
  {
    if (consecutive_fail_count_ > 5)
    {
      XR_LOG_ERROR("WebotsCamera failed repeatedly (%d)!", consecutive_fail_count_);
    }
  }

  void ReportSensorReadFailure(const char* sensor_group, int& fail_count)
  {
    ++fail_count;
    if (fail_count == 1 || fail_count % 1000 == 0)
    {
      XR_LOG_WARN("WebotsCamera: %s sample unavailable (%d consecutive steps)",
                  sensor_group, fail_count);
    }
  }

  void ReportSensorReadRecovered(const char* sensor_group, int& fail_count)
  {
    if (fail_count == 0)
    {
      return;
    }

    XR_LOG_INFO("WebotsCamera: %s sample recovered after %d missed steps",
                sensor_group, fail_count);
    fail_count = 0;
  }

  void ProcessCaptureStep(LibXR::MicrosecondTimestamp timestamp)
  {
    PoseSample pose{};
    if (!ReadCameraPoseSample(pose))
    {
      ReportSensorReadFailure("pose", pose_read_fail_count_);
      return;
    }
    ReportSensorReadRecovered("pose", pose_read_fail_count_);
    PublishGimbalRotation(pose);

    MotionSample motion{};
    if (!ReadImuMotionSample(motion))
    {
      ReportSensorReadFailure("imu motion", motion_read_fail_count_);
      return;
    }
    ReportSensorReadRecovered("imu motion", motion_read_fail_count_);

    PublishRawImu(pose, motion, timestamp);
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

      // Sleep 挂到 libxr Webots timebase；时间戳取同一套仿真时间基。
      LibXR::Thread::Sleep(self->time_step_ms_);

      const LibXR::MicrosecondTimestamp timestamp = LibXR::Timebase::GetMicroseconds();
      const uint64_t step_us = static_cast<uint64_t>(self->time_step_ms_) * 1000ULL;
      const uint64_t step = static_cast<uint64_t>(timestamp) / step_us;
      if (!self->EnterNewStep(step))
      {
        continue;
      }

      self->ProcessCaptureStep(timestamp);
    }
  }

 private:
  int target_fps_ = 30;
  double exposure_ = 1.0;
  double gain_ = 0.0;
  LibXR::RuntimeStringView<> gyro_device_name_{};
  LibXR::RuntimeStringView<> accl_device_name_{};
  LibXR::RuntimeStringView<> quat_device_name_{};
  LibXR::RuntimeStringView<> raw_topic_domain_name_{};
  LibXR::Topic::Domain raw_topic_domain_;
  LibXR::RuntimeStringView<> gyro_topic_name_{};
  LibXR::RuntimeStringView<> accl_topic_name_{};
  LibXR::RuntimeStringView<> quat_topic_name_{};
  LibXR::RuntimeStringView<> trigger_gpio_name_{};
  // rotation topic 保持在 gimbal domain，沿用历史消费者的查找路径。
  LibXR::Topic::Domain gimbal_domain_ = LibXR::Topic::Domain("gimbal");
  LibXR::Topic gimbal_rotation_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation", &gimbal_domain_);
  LibXR::Topic raw_gyro_topic_{};
  LibXR::Topic raw_accl_topic_{};
  LibXR::Topic raw_quat_topic_{};

  webots::Robot* robot_{};
  webots::Camera* cam_ = nullptr;
  webots::Gyro* gyro_ = nullptr;
  webots::Accelerometer* accelerometer_ = nullptr;
  webots::InertialUnit* inertial_unit_ = nullptr;
  int time_step_ms_ = 0;
  int base_image_interval_steps_ = 1;

  std::atomic<bool> running_{false};
  LibXR::Thread capture_thread_{};
  int consecutive_fail_count_ = 0;
  int pose_read_fail_count_ = 0;
  int motion_read_fail_count_ = 0;
  uint64_t last_processed_step_ = std::numeric_limits<uint64_t>::max();
  std::atomic<bool> trigger_level_{false};
  bool trigger_active_level_{true};
  LibXR::GPIO::Configuration trigger_gpio_config_{
      .direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
      .pull = LibXR::GPIO::Pull::NONE};
};
