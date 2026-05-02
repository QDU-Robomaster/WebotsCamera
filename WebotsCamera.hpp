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
#include <string_view>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "CameraBase.hpp"
#include "app_framework.hpp"
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
 * @brief Webots 相机模块。
 *
 * 该模块负责四件事：
 * 1. 从 Webots Camera 取出原始 BGRA 图像；
 * 2. 每个 world step 发布一组原始 imu（gyro/accl/quat）；
 * 3. 按基础分频发布图像，并响应一次性 `sensor_sync_cmd` 探针；
 * 4. 从 Webots IMU 传感器读取姿态与运动量，把图像写入 `CameraBase::ImageFrame`
 *    并交给已注册的图像 sink。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class WebotsCamera : public LibXR::Application,
                     public CameraBase<CameraInfoV>
{
 public:
  using Self = WebotsCamera<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using ImageFrame = typename Base::ImageFrame;
  using GyroStamped = typename Base::GyroStamped;
  using AcclStamped = typename Base::AcclStamped;
  using QuatStamped = typename Base::QuatStamped;
  using SensorSyncCmd = typename Base::SensorSyncCmd;

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
    // 当前发布时刻的相机姿态。rotation 表示对外发布坐标系相对 world 的姿态四元数。
    // 该发布坐标系固定为右手系：x 向右、y 向前、z 向上。
    LibXR::MicrosecondTimestamp timestamp{};
    LibXR::Quaternion<float> rotation{};
    LibXR::Position<float> translation{};
  };

  struct MotionSample
  {
    // 与 PoseSample 同帧下的运动量，分量都落在同一发布坐标系下：
    // x 向右、y 向前、z 向上；角速度正方向遵循各轴右手定则。
    LibXR::Position<float> angular_velocity{};
    LibXR::Position<float> linear_acceleration{};
  };

  struct ImuSensorNames
  {
    // world 里通过统一前缀拼接出来的设备名，例如 camera_gyro。
    LibXR::RuntimeStringView<> gyro;
    LibXR::RuntimeStringView<> accelerometer;
    LibXR::RuntimeStringView<> inertial_unit;
  };

  struct SimClockSample
  {
    // 这里的 step / timestamp 都来自同一次 Webots 仿真时钟采样，避免跨调用抖动。
    uint64_t step = 0;
    LibXR::MicrosecondTimestamp timestamp{};
  };

  struct RuntimeParam
  {
    // Webots Camera 设备名，同时也是原始 topic 前缀；不能为空。
    std::string_view device_name = "camera";

    // 目标发布频率。最终会被量化到世界步长上。
    int fps = 30;

    // Webots 原生支持曝光。
    double exposure = 1.0;

    // Webots 不支持 gain；非零输入会被直接忽略。
    double gain = 0.0;

    // IMU 设备名前缀；不能为空。
    std::string_view pose_def_name = "camera";

    // 图像与 imu 的原始输出名，供同步器和其他消费者复用。
    std::string_view image_topic_name = "camera_image";
    std::string_view imu_topic_name = "camera_imu";
  };

  explicit WebotsCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                        RuntimeParam runtime)
      : Base(hw, runtime.device_name, runtime.image_topic_name, runtime.imu_topic_name),
        target_fps_(runtime.fps),
        exposure_(runtime.exposure),
        gain_(runtime.gain),
        imu_sensor_names_{
            .gyro = LibXR::RuntimeStringView<>(runtime.pose_def_name, "_gyro"),
            .accelerometer =
                LibXR::RuntimeStringView<>(runtime.pose_def_name, "_accelerometer"),
            .inertial_unit =
                LibXR::RuntimeStringView<>(runtime.pose_def_name, "_inertial_unit"),
        },
        gyro_topic_name_(runtime.device_name, "_gyro"),
        accl_topic_name_(runtime.device_name, "_accl"),
        quat_topic_name_(runtime.device_name, "_quat"),
        raw_gyro_topic_(LibXR::Topic::FindOrCreate<GyroStamped>(gyro_topic_name_.CStr())),
        raw_accl_topic_(LibXR::Topic::FindOrCreate<AcclStamped>(accl_topic_name_.CStr())),
        raw_quat_topic_(LibXR::Topic::FindOrCreate<QuatStamped>(quat_topic_name_.CStr())),
        sensor_sync_cmd_topic_(
            LibXR::Topic::FindOrCreate<SensorSyncCmd>("sensor_sync_cmd")),
        sensor_sync_cmd_sub_(sensor_sync_cmd_topic_),
        robot_(_libxr_webots_robot_handle)
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

    InitRobot();
    InitCamera();
    InitImuSensors();
    ValidateCameraGeometry();
    ConfigureSamplingOnStartup();
    ApplyExposure();
    IgnoreUnsupportedGainRequest();
    sensor_sync_cmd_sub_.StartWaiting();
    StartCaptureThread();

    XR_LOG_PASS(
        "Webots camera enabled: name=%s, capture_period=%d ms, world_dt=%d ms, image_divisor=%d",
        this->Name(), base_image_interval_steps_ * time_step_ms_, time_step_ms_,
        base_image_interval_steps_);

    app.Register(*this);
  }

  ~WebotsCamera()
  {
    running_.store(false);
  }

  void OnMonitor() override {}

  void SetExposure(double exposure) override
  {
    exposure_ = exposure;
    ApplyExposure();
  }

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

    time_step_ms_ = static_cast<int>(robot_->getBasicTimeStep());
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
    // 这里只做设备绑定，不做 enable；初始化路径可直接复用这一段。
    gyro_ = robot_->getGyro(imu_sensor_names_.gyro.CStr());
    accelerometer_ = robot_->getAccelerometer(imu_sensor_names_.accelerometer.CStr());
    inertial_unit_ = robot_->getInertialUnit(imu_sensor_names_.inertial_unit.CStr());

    if (gyro_ == nullptr)
    {
      XR_LOG_ERROR("WebotsCamera: gyro '%s' not found in world.",
                   imu_sensor_names_.gyro.CStr());
      throw std::runtime_error("WebotsCamera: gyro device not found");
    }
    if (accelerometer_ == nullptr)
    {
      XR_LOG_ERROR("WebotsCamera: accelerometer '%s' not found in world.",
                   imu_sensor_names_.accelerometer.CStr());
      throw std::runtime_error("WebotsCamera: accelerometer device not found");
    }
    if (inertial_unit_ == nullptr)
    {
      XR_LOG_ERROR("WebotsCamera: inertial unit '%s' not found in world.",
                   imu_sensor_names_.inertial_unit.CStr());
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
                imu_sensor_names_.gyro.CStr(),
                imu_sensor_names_.accelerometer.CStr(),
                imu_sensor_names_.inertial_unit.CStr());
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
    // Webots Camera 的采样周期会直接变成仿真侧渲染负载。
    // 这里把设备采样周期收敛到目标图像周期，避免只想发 100Hz 时仍以 1kHz 渲染。
    base_image_interval_steps_ = ComputePublishIntervalSteps(FpsToPeriodMs(target_fps_),
                                                             time_step_ms_);
    const int camera_sampling_period_ms = base_image_interval_steps_ * time_step_ms_;
    ResetImageSchedule();
    last_processed_step_ = std::numeric_limits<uint64_t>::max();
    cam_->enable(camera_sampling_period_ms);
  }

  void StartCaptureThread()
  {
    // 采集线程把 pose / imu / image 串成同一条发布时间线。
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

    // 调度器复位后的第一张图，等一个完整的 camera 周期后再提交，
    // 避免 Webots 还没产出首帧时先写入共享图像槽位。
    image_schedule_initialized_ = true;
    next_image_step_ = sim_step + static_cast<uint64_t>(base_image_interval_steps_);
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

  void PublishGimbalRotation(const PoseSample& pose)
  {
#if LIBXR_LOG_LEVEL >= 4
    const LibXR::EulerAngle<float> eulr = pose.rotation.ToEulerAngle();
    XR_LOG_DEBUG("WebotsCamera: camera euler roll_x=%.3f pitch_y=%.3f yaw_z=%.3f",
                 eulr.Roll(), eulr.Pitch(), eulr.Yaw());
#endif
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

  bool ReadCameraPoseSample(LibXR::MicrosecondTimestamp timestamp, PoseSample& pose)
  {
    pose.timestamp = timestamp;

    if (inertial_unit_ == nullptr)
    {
      return false;
    }

    const double* raw_xyzw = inertial_unit_->getQuaternion();
    if (raw_xyzw == nullptr)
    {
      return false;
    }

    // Webots InertialUnit::getQuaternion() 返回 xyzw，这里统一转成 libxr 的 wxyz。
    const LibXR::Quaternion<double> raw_quat(raw_xyzw[3], raw_xyzw[0], raw_xyzw[1],
                                             raw_xyzw[2]);
    const LibXR::RotationMatrix<double> camera_rotation_world(raw_quat);

    // 对外发布的 IMU/姿态坐标系固定定义为右手系：x 向右、y 向前、z 向上。
    // 这里的中立矩阵表示“零位时该发布坐标系在 world 下的朝向”。
    // 第一次读到相机姿态时会冻结一份零位标定矩阵，后续所有 rotation /
    // gyro / accel 都统一使用同一份标定，避免中途切换坐标语义。
    static const LibXR::RotationMatrix<double> published_frame_neutral_to_world(
        0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0,
        0.0, 0.0, 1.0);
    if (!pose_zero_calibrated_)
    {
      pose_zero_calibration_ =
          LibXR::RotationMatrix<double>(camera_rotation_world.transpose() *
                                        published_frame_neutral_to_world);
      pose_zero_calibrated_ = true;
      XR_LOG_INFO("WebotsCamera captured published-frame zero calibration");
    }

    const LibXR::Quaternion<double> quat =
        camera_rotation_world * pose_zero_calibration_;
    pose.rotation = LibXR::Quaternion<float>(
        static_cast<float>(quat.w()), static_cast<float>(quat.x()),
        static_cast<float>(quat.y()), static_cast<float>(quat.z()));

    // 当前 WebotsCamera 只负责旋转语义；平移仍维持历史上的零平移发布策略。
    pose.translation = LibXR::Position<float>(0.0f, 0.0f, 0.0f);
    return true;
  }

  LibXR::Position<float> RotateCameraNodeVectorToPublishedFrame(
      const double* raw_xyz) const
  {
    if (raw_xyz == nullptr)
    {
      return LibXR::Position<float>(0.0f, 0.0f, 0.0f);
    }

    const Eigen::Matrix<double, 3, 1> raw_vector(raw_xyz[0], raw_xyz[1], raw_xyz[2]);
    // pose_zero_calibration_ 是 “camera node 局部轴 -> 对外发布坐标系” 的零位修正。
    // 对向量做 transpose，等价于把原始 camera node 局部向量旋到发布坐标系
    // （x 向右、y 向前、z 向上）下。
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

    // Webots 传感器原始值位于 camera node 局部轴；这里统一旋到对外发布坐标系
    // （x 向右、y 向前、z 向上），确保 rotation / gyro / accel 三者坐标语义一致。
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

  void ProcessCaptureStep(const SimClockSample& clock)
  {
    InitializeImageScheduleIfNeeded(clock.step);
    PollSensorSyncProbeCommand();

    PoseSample pose{};
    if (!ReadCameraPoseSample(clock.timestamp, pose))
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

    PublishRawImu(pose, motion, clock.timestamp);
    if (!ShouldPublishImageAtStep(clock.step))
    {
      return;
    }

    AdvanceImageScheduleAfterPublish();
    CommitImageSample(clock.timestamp);
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

      // Webots 控制器线程不直接调用 robot->step；Sleep 在 Webots 后端会挂到
      // step 通知上，libxr 时间基会等 REALTIME 采集线程再次挂起后才进入下一步。
      LibXR::Thread::Sleep(self->time_step_ms_);

      const SimClockSample clock = self->ReadSimClockSample();
      if (!self->EnterNewStep(clock.step))
      {
        continue;
      }

      self->ProcessCaptureStep(clock);
    }
  }

 private:
  int target_fps_ = 30;
  double exposure_ = 1.0;
  double gain_ = 0.0;
  ImuSensorNames imu_sensor_names_{};
  LibXR::RuntimeStringView<> gyro_topic_name_{};
  LibXR::RuntimeStringView<> accl_topic_name_{};
  LibXR::RuntimeStringView<> quat_topic_name_{};
  // rotation topic 保持在 gimbal domain，沿用历史消费者的查找路径。
  LibXR::Topic::Domain gimbal_domain_ = LibXR::Topic::Domain("gimbal");
  LibXR::Topic gimbal_rotation_topic_ =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation", &gimbal_domain_);
  LibXR::Topic raw_gyro_topic_{};
  LibXR::Topic raw_accl_topic_{};
  LibXR::Topic raw_quat_topic_{};
  LibXR::Topic sensor_sync_cmd_topic_{};
  LibXR::Topic::ASyncSubscriber<SensorSyncCmd> sensor_sync_cmd_sub_;

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
  uint64_t next_image_step_{0};
  bool next_image_interval_stretched_{false};
  bool image_schedule_initialized_{false};
  bool pose_zero_calibrated_ = false;
  // 首帧冻结的零位标定矩阵：把 raw camera node 姿态/向量对齐到外部发布帧。
  LibXR::RotationMatrix<double> pose_zero_calibration_{};
};
