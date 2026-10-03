#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Webots 仿真中的相机与 IMU 传感器端点：发布原始 IMU 数据，并在触发 GPIO 有效时提交图像 / Camera and IMU sensor endpoint in Webots simulation that publishes raw IMU data and commits images when the trigger GPIO becomes active
depends:
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <span>
#include <stdexcept>
#include <string_view>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Robot.hpp>

#include "CameraBase.hpp"
#include "gpio.hpp"
#include "libxr.hpp"
#include "libxr_string.hpp"
#include "libxr_system.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "transform.hpp"

extern webots::Robot* _libxr_webots_robot_handle;

/**
 * @brief Webots 仿真环境中的相机与 IMU 数据源。
 *        Camera and IMU data source in the Webots simulation.
 *
 * @details 按 Webots step 读取 `Gyro`、`Accelerometer` 和 `InertialUnit`，把原始 IMU
 *          样本发布到 `<device_name>_gyro`、`<device_name>_accl`、`<device_name>_quat`。
 *          模块实现 `LibXR::GPIO`，由 CameraSync 像真实 MCU 一样翻转触发线，
 *          进入有效电平的边沿提交一帧图像。同步命令、分频拉长、seq 回执和 Host/MCU
 *          SharedTopic 边界由 CameraSync 与 CameraFrameSync 负责。
 *          Reads `Gyro`, `Accelerometer` and `InertialUnit` every Webots step and
 *          publishes the raw IMU samples to `<device_name>_gyro`, `<device_name>_accl`
 *          and `<device_name>_quat`. The Module implements `LibXR::GPIO`; CameraSync
 *          toggles the trigger line like a real MCU, and the edge entering the active
 *          level commits one image. The synchronization commands, the divider stretching,
 *          the seq acknowledgements and the Host/MCU SharedTopic boundary are handled by
 *          CameraSync and CameraFrameSync.
 *
 * @tparam FrameLayoutV 编译期图像存储布局，紧密排列的 BGR8。
 *                      Image storage layout at compile time, packed BGR8.
 */
template <CameraTypes::FrameLayout FrameLayoutV>
class WebotsCamera : public CameraBase<FrameLayoutV>, public LibXR::GPIO
{
 public:
  using Self = WebotsCamera<FrameLayoutV>;
  using Base = CameraBase<FrameLayoutV>;
  using CameraCalibration = typename Base::CameraCalibration;
  using CameraProfile = typename Base::CameraProfile;
  using AppliedProfile = typename Base::AppliedProfile;
  using ProfileId = typename Base::ProfileId;
  using ImageFrame = typename Base::ImageFrame;
  using ImuSample = Eigen::Matrix<float, 3, 1>;
  using QuatSample = LibXR::Quaternion<float>;

  static inline constexpr auto frame_layout = Base::frame_layout;
  static constexpr int channel_count = 3;
  static constexpr int frame_width = static_cast<int>(frame_layout.width);
  static constexpr int frame_height = static_cast<int>(frame_layout.height);
  static constexpr std::size_t frame_step = static_cast<std::size_t>(frame_layout.step);
  static inline constexpr CameraTypes::FrameGeometry frame_geometry{
      .width = frame_layout.width,
      .height = frame_layout.height,
      .step = frame_layout.step,
      .roi_offset_x_native = 0,
      .roi_offset_y_native = 0,
      .decimation_x = 1,
      .decimation_y = 1};

  static_assert(frame_layout.encoding == CameraTypes::Encoding::BGR8,
                "WebotsCamera requires BGR8 output encoding");
  static_assert(frame_step ==
                    static_cast<std::size_t>(frame_layout.width) * channel_count,
                "WebotsCamera requires packed BGR step");

  /**
   * @brief 单个 Webots step 内的姿态样本。
   *        Attitude sample of one Webots step.
   */
  struct PoseSample
  {
    LibXR::Quaternion<float> rotation{};  ///< 发布坐标系相对 world 的姿态，wxyz
    ///< Attitude of the published frame relative to the world, wxyz
    LibXR::Position<float> translation{};  ///< 相机平移，固定为 0
    ///< Camera translation, fixed at 0
  };

  /**
   * @brief 单个 Webots step 内的运动样本。
   *        Motion sample of one Webots step.
   */
  struct MotionSample
  {
    LibXR::Position<float> angular_velocity{};  ///< 角速度，单位 rad/s
    ///< Angular velocity in rad/s
    LibXR::Position<float> linear_acceleration{};  ///< 线加速度，单位 m/s^2
    ///< Linear acceleration in m/s^2
  };

  /**
   * @brief 运行时参数，由 xrobot YAML 传入。
   *        Runtime parameters passed from the xrobot YAML.
   */
  struct RuntimeParam
  {
    std::string_view device_name = "camera";  ///< Camera 设备名与原始 IMU Topic 前缀
    ///< Webots Camera device name and raw IMU Topic prefix
    int fps = 30;  ///< 图像采样频率，实际周期量化到 Webots step
    ///< Image sampling rate, the actual period is quantized to Webots steps
    double exposure = 1.0;  ///< Webots Camera 曝光值
    ///< Webots Camera exposure value
    double gain = 0.0;  ///< 增益；Webots Camera 没有增益控制，非零值记录警告后清零
    ///< Gain; the Webots Camera has no gain control, a non-zero value logs a warning and
    ///< is reset to zero
    std::string_view pose_def_name = "camera";  ///< IMU 设备名前缀
    ///< IMU device name prefix
    std::string_view image_topic_name = "camera_image";  ///< 原始图像 Topic 名
    ///< Raw image Topic name
    std::string_view imu_topic_name = "camera_imu";  ///< 同步 IMU Topic 名
    ///< Synchronized IMU Topic name
    std::string_view raw_topic_domain_name = "mcu";  ///< 原始 IMU Topic 的 domain
    ///< Domain of the raw IMU Topics; the default domain can be used for in-process
    ///< debugging
    bool trigger_active_level = true;  ///< 触发有效电平，进入该电平的边沿提交图像
    ///< Active trigger level; the edge entering this level commits an image
    uint32_t trigger_period_us = 20000;  ///< 固定 WIDE 档位的触发周期，单位 us
    ///< Trigger period of the fixed WIDE profile in us
  };

  /**
   * @brief 获取默认标定：1280x720、`fx = fy = 800`、主点 (640, 360)、零畸变。
   *        Get the default calibration: 1280x720, `fx = fy = 800`, principal point (640,
   *        360), zero distortion.
   *
   * @return 默认的原生相机标定。
   *         The default native camera calibration.
   */
  static CameraCalibration DefaultCalibration() { return {.native_width = 1280, .native_height = 720, .camera_matrix = {800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0}, .distortion_model = CameraTypes::DistortionModel::PLUMB_BOB, .distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0}, .rectification_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, .projection_matrix = {800.0, 0.0, 640.0, 0.0, 0.0, 800.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0}}; }

  /**
   * @brief 获取默认运行时参数。
   *        Get the default runtime parameters.
   *
   * @return 各项取默认值的运行时参数。
   *         Runtime parameters with all defaults.
   */
  static RuntimeParam DefaultRuntime() { return {}; }

  /**
   * @brief 构造 WebotsCamera：校验参数、绑定 Webots 设备并启动采集线程。
   *        Construct WebotsCamera: validate the parameters, bind the Webots devices and
   *        start the capture thread.
   *
   * @param ramfs 注册相机命令文件的 RamFS。
   *              RamFS that registers the camera command file.
   * @param calibration 原生相机标定，尺寸等于帧布局。
   *                    Native camera calibration whose size equals the frame layout.
   * @param runtime 运行时参数。
   *                Runtime parameters.
   *
   * @note 参数不合法时抛出 `std::invalid_argument`；设备缺失或几何不一致时抛出
   *       `std::runtime_error`。
   *       Throws `std::invalid_argument` for invalid parameters and `std::runtime_error`
   *       for missing devices or a geometry mismatch.
   */
  explicit WebotsCamera(
      LibXR::RamFS& ramfs,
      CameraCalibration calibration = DefaultCalibration(),
      RuntimeParam runtime = DefaultRuntime())
      : Base(ramfs, calibration, runtime.device_name, runtime.image_topic_name,
             runtime.imu_topic_name),
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
        raw_gyro_topic_(LibXR::Topic::FindOrCreate<ImuSample>(gyro_topic_name_.CStr(),
                                                              &raw_topic_domain_)),
        raw_accl_topic_(LibXR::Topic::FindOrCreate<ImuSample>(accl_topic_name_.CStr(),
                                                              &raw_topic_domain_)),
        raw_quat_topic_(LibXR::Topic::FindOrCreate<QuatSample>(quat_topic_name_.CStr(),
                                                               &raw_topic_domain_)),
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
    if (runtime.raw_topic_domain_name.empty())
    {
      XR_LOG_ERROR("WebotsCamera: runtime sync names must not be empty");
      throw std::invalid_argument("WebotsCamera: sync runtime string is empty");
    }

    if (runtime.trigger_period_us == 0 ||
        calibration.native_width != frame_layout.width ||
        calibration.native_height != frame_layout.height)
    {
      throw std::invalid_argument(
          "WebotsCamera requires a nonzero trigger period and native frame calibration");
    }
    profiles_[0] = {ProfileId::WIDE, frame_geometry, runtime.trigger_period_us};

    InitRobot();
    InitCamera();
    InitImuSensors();
    ValidateCameraGeometry();
    ConfigureSamplingOnStartup();
    ApplyExposure();
    IgnoreUnsupportedGainRequest();
    StartCaptureThread();

    XR_LOG_PASS(
        "Webots camera enabled: name=%s, capture_period=%d ms, world_dt=%d ms, "
        "image_divisor=%d",
        this->Name(), base_image_interval_steps_ * time_step_ms_, time_step_ms_,
        base_image_interval_steps_);
  }

  /**
   * @brief 请求采集线程退出。
   *        Request the capture thread to exit.
   */
  ~WebotsCamera() { running_.store(false); }

  /**
   * @brief 获取档位列表，包含一个固定的 WIDE 档位。
   *        Get the profile list, which holds one fixed WIDE profile.
   *
   * @return 档位的只读视图。
   *         Read-only view of the profiles.
   */
  [[nodiscard]] std::span<const CameraProfile> Profiles() const noexcept override
  {
    return profiles_;
  }

  /**
   * @brief 切换档位，仅支持 WIDE。
   *        Switch the profile; only WIDE is supported.
   *
   * @param id 目标档位。
   *           Target profile.
   * @param applied 生效的档位与几何。
   *                Applied profile and geometry.
   * @return WIDE 为 `ErrorCode::OK`，其他档位为 `ErrorCode::NOT_SUPPORT`。
   *         `ErrorCode::OK` for WIDE and `ErrorCode::NOT_SUPPORT` for other profiles.
   */
  LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) override
  {
    if (id != ProfileId::WIDE)
    {
      return LibXR::ErrorCode::NOT_SUPPORT;
    }
    applied = {ProfileId::WIDE, frame_geometry};
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 读取仿真触发线的电平。
   *        Read the level of the simulated trigger line.
   *
   * @return 当前触发线电平。
   *         Current trigger line level.
   */
  bool Read() override { return trigger_level_.load(std::memory_order_relaxed); }

  /**
   * @brief 写入仿真触发线；进入有效电平的边沿记录触发时间，之后的采集 step 提交图像。
   *        Write the simulated trigger line; the edge entering the active level records
   *        the trigger time, and a following capture step commits the image.
   *
   * @details CameraSync 在 MCU 侧的 IMU 回调中翻转该 GPIO，行为与真实相机触发脚一致。
   *          CameraSync toggles this GPIO in the MCU-side IMU callback, matching the
   *          behavior of a real camera trigger pin.
   *
   * @param value 触发线电平。
   *              Trigger line level.
   */
  void Write(bool value) override
  {
    const bool old = trigger_level_.exchange(value, std::memory_order_relaxed);
    if (old == value || value != trigger_active_level_)
    {
      return;
    }

    uint64_t timestamp_us = current_raw_imu_timestamp_us_.load(std::memory_order_acquire);
    if (timestamp_us == 0ULL)
    {
      timestamp_us = static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds());
    }
    pending_trigger_timestamp_us_.store(timestamp_us, std::memory_order_release);
  }

  /**
   * @brief 使能中断，Webots 仿真 GPIO 直接返回成功。
   *        Enable the interrupt; the Webots simulated GPIO returns success directly.
   *
   * @return `ErrorCode::OK`。
   *         `ErrorCode::OK`.
   */
  LibXR::ErrorCode EnableInterrupt() override { return LibXR::ErrorCode::OK; }

  /**
   * @brief 禁用中断，Webots 仿真 GPIO 直接返回成功。
   *        Disable the interrupt; the Webots simulated GPIO returns success directly.
   *
   * @return `ErrorCode::OK`。
   *         `ErrorCode::OK`.
   */
  LibXR::ErrorCode DisableInterrupt() override { return LibXR::ErrorCode::OK; }

  /**
   * @brief 保存 GPIO 配置，用于调试时确认 CameraSync 配置过该触发脚。
   *        Store the GPIO configuration so that a debug session can confirm that
   *        CameraSync configured this trigger pin.
   *
   * @param config GPIO 配置。
   *               GPIO configuration.
   * @return `ErrorCode::OK`。
   *         `ErrorCode::OK`.
   */
  LibXR::ErrorCode SetConfig(Configuration config) override
  {
    trigger_gpio_config_ = config;
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 更新 Webots Camera 的曝光值。
   *        Update the Webots Camera exposure.
   *
   * @param exposure 曝光值。
   *                 Exposure value.
   */
  void SetExposure(double exposure) override
  {
    exposure_ = exposure;
    ApplyExposure();
  }

  /**
   * @brief 设置增益；Webots Camera 没有增益控制，非零值记录警告后清零。
   *        Set the gain; the Webots Camera has no gain control, so a non-zero value logs
   *        a warning and is reset to zero.
   *
   * @param gain 增益。
   *             Gain value.
   */
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

    if (frame_layout.width == actual_width && frame_layout.height == actual_height &&
        frame_layout.step == packed_step)
    {
      return;
    }

    XR_LOG_ERROR(
        "WebotsCamera: constexpr geometry mismatch width=%u/%u height=%u/%u step=%u/%u",
        frame_layout.width, actual_width, frame_layout.height, actual_height,
        frame_layout.step, packed_step);
    throw std::runtime_error("WebotsCamera: constexpr geometry mismatch");
  }

  void ConfigureSamplingOnStartup()
  {
    // Camera enable 周期只决定 Webots 渲染负载；真正提交图像由 GPIO 触发边沿决定。
    base_image_interval_steps_ =
        ComputePublishIntervalSteps(FpsToPeriodMs(target_fps_), time_step_ms_);
    const int camera_sampling_period_ms = base_image_interval_steps_ * time_step_ms_;
    last_processed_step_ = std::numeric_limits<uint64_t>::max();
    cam_->enable(camera_sampling_period_ms);
  }

  void StartCaptureThread()
  {
    // 采集线程参与 Webots step 栅栏，保证 IMU / image 传感器时间和仿真步一致。
    // 图像话题的订阅回调（CameraFrameSync、ArmorDetector 预处理等）在本线程同步执行，
    // 栈与实机相机线程（std::thread 默认 8 MiB）保持一致。
    // Topic subscribers (CameraFrameSync, ArmorDetector preprocessing) run on this
    // thread, so it gets the same stack as the hardware camera's std::thread (8 MiB).
    constexpr size_t capture_stack_bytes = 8U * 1024U * 1024U;
    running_.store(true);
    capture_thread_.Create<Self*>(this, CaptureThreadMain, "webots_camera",
                                  capture_stack_bytes, LibXR::Thread::Priority::REALTIME);
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

  static LibXR::Quaternion<float> WebotsRawImuToPublicBodyQuat(
      const LibXR::Quaternion<float>& sensor)
  {
    // The current Webots IMU mount reports +X as camera-down. Convert it to
    // the public body frame B, where +X roll raises the forward optical ray.
    const LibXR::Quaternion<float> webots_raw_to_body(0.0f, 0.0f, 0.0f, 1.0f);
    const LibXR::Quaternion<float> body_to_webots_raw(0.0f, 0.0f, 0.0f, -1.0f);
    return webots_raw_to_body * sensor * body_to_webots_raw;
  }

  static LibXR::Position<float> WebotsRawImuToPublicBodyVector(
      const LibXR::Position<float>& vector)
  {
    return LibXR::Position<float>(-vector[0], -vector[1], vector[2]);
  }

  void PublishGimbalQuat(const PoseSample& pose, LibXR::MicrosecondTimestamp timestamp)
  {
#if LIBXR_LOG_LEVEL >= 4
    const LibXR::EulerAngle<float> eulr = pose.rotation.ToEulerAngle();
    XR_LOG_DEBUG("WebotsCamera: camera euler roll_x=%.3f pitch_y=%.3f yaw_z=%.3f",
                 eulr.Roll(), eulr.Pitch(), eulr.Yaw());
#endif
    auto rotation = WebotsRawImuToPublicBodyQuat(pose.rotation);
    gimbal_quat_topic_.Publish(rotation, timestamp);
  }

  void PublishRawImu(const PoseSample& pose, const MotionSample& motion,
                     LibXR::MicrosecondTimestamp timestamp)
  {
    current_raw_imu_timestamp_us_.store(static_cast<uint64_t>(timestamp),
                                        std::memory_order_release);

    const auto angular_velocity = WebotsRawImuToPublicBodyVector(motion.angular_velocity);
    const auto linear_acceleration =
        WebotsRawImuToPublicBodyVector(motion.linear_acceleration);
    const auto rotation = WebotsRawImuToPublicBodyQuat(pose.rotation);

    ImuSample gyro(angular_velocity[0], angular_velocity[1], angular_velocity[2]);
    ImuSample accl(linear_acceleration[0], linear_acceleration[1],
                   linear_acceleration[2]);
    QuatSample quat(rotation.w(), rotation.x(), rotation.y(), rotation.z());

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

    // 平移固定为 0，由下游静态外参提供。
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

    // 坐标轴由 world 中传感器节点的安装方向决定。
    motion.angular_velocity = LibXR::Position<float>(
        static_cast<float>(angular_velocity[0]), static_cast<float>(angular_velocity[1]),
        static_cast<float>(angular_velocity[2]));
    motion.linear_acceleration =
        LibXR::Position<float>(static_cast<float>(linear_acceleration[0]),
                               static_cast<float>(linear_acceleration[1]),
                               static_cast<float>(linear_acceleration[2]));
    return true;
  }

  // ---- 帧写入与提交 ----

  bool WriteAndCommitImage(const unsigned char* rgba,
                           LibXR::MicrosecondTimestamp timestamp)
  {
    ImageFrame* image = this->GetWritableImage();
    if (image == nullptr)
    {
      consecutive_fail_count_++;
      XR_LOG_WARN("WebotsCamera: writable image is null.");
      return false;
    }

    image->timestamp_us = timestamp;
    image->geometry = frame_geometry;

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

    XR_LOG_INFO("WebotsCamera: %s sample recovered after %d missed steps", sensor_group,
                fail_count);
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
    PublishGimbalQuat(pose, timestamp);

    MotionSample motion{};
    if (!ReadImuMotionSample(motion))
    {
      ReportSensorReadFailure("imu motion", motion_read_fail_count_);
      return;
    }
    ReportSensorReadRecovered("imu motion", motion_read_fail_count_);

    PublishRawImu(pose, motion, timestamp);
    CommitTriggeredImageIfReady(timestamp);
  }

  void CommitTriggeredImageIfReady(LibXR::MicrosecondTimestamp timestamp)
  {
    const uint64_t current_timestamp_us = static_cast<uint64_t>(timestamp);
    uint64_t trigger_timestamp_us =
        pending_trigger_timestamp_us_.load(std::memory_order_acquire);
    if (trigger_timestamp_us == 0ULL || trigger_timestamp_us > current_timestamp_us)
    {
      return;
    }

    if (!pending_trigger_timestamp_us_.compare_exchange_strong(trigger_timestamp_us, 0ULL,
                                                               std::memory_order_acq_rel,
                                                               std::memory_order_acquire))
    {
      return;
    }

    CommitImageSample(static_cast<LibXR::MicrosecondTimestamp>(trigger_timestamp_us));
    ReportRepeatedFailureIfNeeded();
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

      const uint64_t step_us = static_cast<uint64_t>(self->time_step_ms_) * 1000ULL;
      const auto webots_time_us =
          static_cast<uint64_t>(std::llround(self->robot_->getTime() * 1000000.0));
      const uint64_t step = webots_time_us / step_us;
      if (!self->EnterNewStep(step))
      {
        continue;
      }

      self->ProcessCaptureStep(static_cast<LibXR::MicrosecondTimestamp>(webots_time_us));
    }
  }

 private:
  std::array<CameraProfile, 1> profiles_{};
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
  LibXR::Topic::Domain host_domain_ = LibXR::Topic::Domain("host");
  LibXR::Topic gimbal_quat_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("gimbal_quat", &host_domain_);
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
  std::atomic<uint64_t> pending_trigger_timestamp_us_{0};
  std::atomic<uint64_t> current_raw_imu_timestamp_us_{0};
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
