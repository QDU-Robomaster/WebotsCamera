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
#include "linux_shared_topic.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "timer.hpp"
#include "transform.hpp"

#include <webots/Camera.hpp>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

/**
 * @brief Webots 相机模块。
 *
 * 这个模块做三件事：
 * 1. 从 Webots Camera 取出 `BGRA` 图像并转换为 `BGR8`
 * 2. 按配置帧率节流后发布到共享内存图像话题
 * 3. 通过 Supervisor 读取相机位姿，并同步发布云台旋转信息
 *
 * 设计边界：
 * - 只服务当前 Webots 仿真链路，不追求泛化成真实硬件相机驱动
 * - 编译期相机模型由 `CameraInfoV` 提供，运行期只允许调整频率和少量参数
 */
template <CameraTypes::CameraInfo CameraInfoV>
class WebotsCamera : public LibXR::Application, public CameraBase<CameraInfoV>
{
 public:
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using SharedImageFrame = typename Base::SharedImageFrame;
  using SharedImageTopic = LibXR::LinuxSharedTopic<SharedImageFrame>;

  // 编译期相机静态信息，后续所有尺寸与步长都从这里派生。
  static inline constexpr CameraInfo camera_info = Base::camera_info;
  // Webots `Camera::getImage()` 返回的是 BGRA。
  static constexpr int channel_count = 3;
  static constexpr int frame_width = static_cast<int>(camera_info.width);
  static constexpr int frame_height = static_cast<int>(camera_info.height);
  static constexpr std::size_t frame_step = static_cast<std::size_t>(camera_info.step);
  // 图像共享话题固定采用 4 个槽位，满足当前单发布者链路。
  static constexpr LibXR::LinuxSharedTopicConfig image_topic_config{
      .slot_num = 4,
      .subscriber_num = 2,
      .queue_num = 4,
  };

  static_assert(camera_info.encoding == CameraTypes::Encoding::BGR8,
                "WebotsCamera requires BGR8 output encoding");
  static_assert(frame_step == static_cast<std::size_t>(camera_info.width) * channel_count,
                "WebotsCamera requires packed BGR step");

  /**
   * @brief Webots 相机位姿消息。
   *
   * 这里只保留当前链路真正消费的最小信息：时间戳、旋转、平移。
   */
  struct PoseStamped
  {
    LibXR::MicrosecondTimestamp timestamp{};
    LibXR::Quaternion<float> rotation{};
    LibXR::Position<float> translation{};
  };

  /**
   * @brief 运行时参数。
   *
   * - `device_name` 对应 Webots 设备名，用于 `robot_->getCamera(...)`
   * - `pose_def_name` 对应场景树里的 DEF 名，用于 Supervisor 查位姿
   */
  struct RuntimeParam
  {
    const char* device_name = "camera";
    int fps = 30;
    double exposure = 1.0;
    double gain = 0.0;
    std::string image_topic_name = "image_frame";
    std::string pose_def_name = "camera";
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
  static const char* SafeCStr(const char* text);
  static bool SameCStr(const char* lhs, const char* rhs);
  void UpdateParameters();
  void EnsureCameraNode();
  bool ShouldPublishFrame(uint64_t sim_step);
  void PublishPoseToGimbalTopic();
  void PublishImageFrame(const unsigned char* rgba, LibXR::MicrosecondTimestamp timestamp);
  LibXR::MicrosecondTimestamp CurrentSimTimeUs() const;
  uint64_t CurrentSimStep() const;
  static void ThreadFun(WebotsCamera<CameraInfoV>* self);
  static int FpsToPeriodMS(int fps, int fallback_ms);
  PoseStamped ReadCameraPoseStamped(LibXR::MicrosecondTimestamp timestamp) const;

 private:
  // 运行时配置副本。字符串字段使用自持有对象，避免悬空指针。
  RuntimeParam runtime_{};
  // 图像通过 Linux 共享内存话题发布，降低后级复制开销。
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

// Template implementation stays in-header so every instantiation sees the full
// definition without a separate `.inl`/`.cpp` split.

template <CameraTypes::CameraInfo CameraInfoV>
int WebotsCamera<CameraInfoV>::FpsToPeriodMS(int fps, int fallback_ms)
{
  if (fps <= 0)
  {
    return fallback_ms;
  }
  const double period = 1000.0 / static_cast<double>(fps);
  int ms = static_cast<int>(std::lround(period));
  return ms < 1 ? 1 : ms;
}

template <CameraTypes::CameraInfo CameraInfoV>
const char* WebotsCamera<CameraInfoV>::SafeCStr(const char* text)
{
  return text != nullptr ? text : "";
}

template <CameraTypes::CameraInfo CameraInfoV>
bool WebotsCamera<CameraInfoV>::SameCStr(const char* lhs, const char* rhs)
{
  return std::strcmp(SafeCStr(lhs), SafeCStr(rhs)) == 0;
}

template <CameraTypes::CameraInfo CameraInfoV>
LibXR::MicrosecondTimestamp WebotsCamera<CameraInfoV>::CurrentSimTimeUs() const
{
  const double sim_time_s = robot_ != nullptr ? std::max(0.0, robot_->getTime()) : 0.0;
  return LibXR::MicrosecondTimestamp(
      static_cast<uint64_t>(std::llround(sim_time_s * 1000000.0)));
}

template <CameraTypes::CameraInfo CameraInfoV>
uint64_t WebotsCamera<CameraInfoV>::CurrentSimStep() const
{
  if (time_step_ms_ <= 0)
  {
    return 0;
  }

  const double sim_time_s = robot_ != nullptr ? std::max(0.0, robot_->getTime()) : 0.0;
  const uint64_t sim_time_ms = static_cast<uint64_t>(std::llround(sim_time_s * 1000.0));
  return sim_time_ms / static_cast<uint64_t>(time_step_ms_);
}

template <CameraTypes::CameraInfo CameraInfoV>
void WebotsCamera<CameraInfoV>::EnsureCameraNode()
{
  if (cam_node_ == nullptr && supervisor_ != nullptr)
  {
    cam_node_ = supervisor_->getFromDef(runtime_.pose_def_name.c_str());
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
bool WebotsCamera<CameraInfoV>::ShouldPublishFrame(uint64_t sim_step)
{
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

template <CameraTypes::CameraInfo CameraInfoV>
void WebotsCamera<CameraInfoV>::PublishPoseToGimbalTopic()
{
  EnsureCameraNode();
  if (cam_node_ == nullptr)
  {
    return;
  }

  auto pose = ReadCameraPoseStamped(CurrentSimTimeUs());
  LibXR::EulerAngle<float> eulr = pose.rotation.ToEulerAngle();
  XR_LOG_DEBUG("WebotsCamera: camera eulr: %.3f, %.3f, %.3f", eulr.Roll(),
               eulr.Pitch(), eulr.Yaw());
  gimbal_rotation_topic_.Publish(pose.rotation);
}

template <CameraTypes::CameraInfo CameraInfoV>
void WebotsCamera<CameraInfoV>::PublishImageFrame(
    const unsigned char* rgba, LibXR::MicrosecondTimestamp timestamp)
{
  typename SharedImageTopic::Data shared_image_data;
  const auto create_ans = image_frame_topic_.CreateData(shared_image_data);
  if (create_ans != LibXR::ErrorCode::OK)
  {
    fail_count_++;
    XR_LOG_WARN("WebotsCamera: shared image slot unavailable (err=%d).",
                static_cast<int>(create_ans));
    return;
  }

  SharedImageFrame* shared_frame = shared_image_data.GetData();
  if (shared_frame == nullptr)
  {
    fail_count_++;
    XR_LOG_WARN("WebotsCamera: shared image slot returned null payload.");
    return;
  }

  cv::Mat src(frame_height, frame_width, CV_8UC4, const_cast<unsigned char*>(rgba));
  cv::Mat dst(frame_height, frame_width, CV_8UC3, shared_frame->data.data(), frame_step);
  cv::cvtColor(src, dst, cv::COLOR_BGRA2BGR);

  shared_frame->timestamp_us = static_cast<uint64_t>(timestamp);
  shared_frame->sequence = frame_sequence_++;

  auto pose = ReadCameraPoseStamped(timestamp);
  camera_pose_topic_.Publish(pose);

  const auto publish_ans = image_frame_topic_.Publish(shared_image_data);
  if (publish_ans != LibXR::ErrorCode::OK)
  {
    fail_count_++;
    XR_LOG_WARN("WebotsCamera: shared image publish failed (err=%d).",
                static_cast<int>(publish_ans));
    return;
  }

  fail_count_ = 0;
}

template <CameraTypes::CameraInfo CameraInfoV>
WebotsCamera<CameraInfoV>::WebotsCamera(LibXR::HardwareContainer& hw,
                                        LibXR::ApplicationManager& app,
                                        RuntimeParam runtime)
    : Base(hw, runtime.device_name),
      runtime_(runtime),
      image_frame_topic_(runtime_.image_topic_name.c_str(), image_topic_config),
      robot_(_libxr_webots_robot_handle)
{
  XR_LOG_INFO("Starting WebotsCamera!");

  // 1. 绑定 Webots 机器人句柄与基础时间步长。
  if (!robot_)
  {
    XR_LOG_ERROR("Webots robot handle is null!");
    std::exit(-1);
  }
  time_step_ms_ = static_cast<int>(robot_->getBasicTimeStep());
  if (time_step_ms_ <= 0)
  {
    time_step_ms_ = 16;
  }

  // 2. 绑定相机设备，并校验运行时设备分辨率与编译期模型一致。
  cam_ = robot_->getCamera(runtime_.device_name);
  if (!cam_)
  {
    XR_LOG_ERROR("Webots Camera '%s' not found!", runtime_.device_name);
    throw std::runtime_error("WebotsCamera: camera device not found");
  }

  const uint32_t actual_width = static_cast<uint32_t>(cam_->getWidth());
  const uint32_t actual_height = static_cast<uint32_t>(cam_->getHeight());
  const uint32_t packed_step = actual_width * channel_count;
  if (camera_info.width != actual_width || camera_info.height != actual_height ||
      camera_info.step != packed_step)
  {
    XR_LOG_ERROR(
        "WebotsCamera: constexpr geometry mismatch width=%u/%u height=%u/%u step=%u/%u",
        camera_info.width, actual_width, camera_info.height, actual_height,
        camera_info.step, packed_step);
    throw std::runtime_error("WebotsCamera: constexpr geometry mismatch");
  }

  // 3. 根据目标帧率换算采样周期，并计算发布节流步数。
  sample_period_ms_ = FpsToPeriodMS(runtime_.fps, 33);
  publish_interval_steps_ =
      std::max(1, static_cast<int>(std::lround(static_cast<double>(sample_period_ms_) /
                                               static_cast<double>(time_step_ms_))));
  cam_->enable(sample_period_ms_);

  // 4. 应用初始曝光，并拉起取图线程。
  cam_->setExposure(runtime_.exposure);
  XR_LOG_INFO("Initial exposure set to %.3f", runtime_.exposure);

  running_.store(true);
  capture_thread_.Create(this, ThreadFun, "WebotsCameraThread",
                         static_cast<size_t>(1024 * 128),
                         LibXR::Thread::Priority::REALTIME);

  XR_LOG_PASS(
      "Webots camera enabled: name=%s, period=%d ms, world_dt=%d ms, publish_every=%d step(s)",
      runtime_.device_name, sample_period_ms_, time_step_ms_, publish_interval_steps_);

  // 5. 绑定 Supervisor，用定时器持续发布相机姿态对应的旋转话题。
  supervisor_ = hw.FindOrExit<webots::Supervisor>({"supervisor"});

  auto timer_handle = LibXR::Timer::CreateTask<WebotsCamera<CameraInfoV>*>(
      [](WebotsCamera<CameraInfoV>* self)
      {
        self->PublishPoseToGimbalTopic();
      },
      this, 1);

  LibXR::Timer::Add(timer_handle);
  LibXR::Timer::Start(timer_handle);

  app.Register(*this);
}

template <CameraTypes::CameraInfo CameraInfoV>
WebotsCamera<CameraInfoV>::~WebotsCamera()
{
  running_.store(false);

  if (cam_)
  {
    cam_->disable();
    cam_ = nullptr;
  }

  XR_LOG_INFO("WebotsCamera destroyed!");
}

template <CameraTypes::CameraInfo CameraInfoV>
typename WebotsCamera<CameraInfoV>::PoseStamped WebotsCamera<CameraInfoV>::ReadCameraPoseStamped(
    LibXR::MicrosecondTimestamp timestamp) const
{
  typename WebotsCamera<CameraInfoV>::PoseStamped pose{};
  pose.timestamp = timestamp;

  auto* mutable_self = const_cast<WebotsCamera<CameraInfoV>*>(this);
  mutable_self->EnsureCameraNode();
  if (cam_node_ == nullptr)
  {
    return pose;
  }

  // Webots 相机坐标系与当前自瞄链路约定不同，这里做一次固定补偿。
  static const LibXR::RotationMatrix<double> compensation =
      LibXR::RotationMatrix<double>(0, 1, 0, 1, 0, 0, 0, 0, -1);

  const double* r9_cam = cam_node_->getOrientation();
  LibXR::RotationMatrix<double> r_final =
      LibXR::RotationMatrix<double>(r9_cam[0], r9_cam[1], r9_cam[2], r9_cam[3],
                                    r9_cam[4], r9_cam[5], r9_cam[6], r9_cam[7],
                                    r9_cam[8]) *
      compensation;

  LibXR::Quaternion<double> quat = r_final;
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

template <CameraTypes::CameraInfo CameraInfoV>
void WebotsCamera<CameraInfoV>::SetRuntimeParam(const RuntimeParam& p)
{
  const bool topic_changed = runtime_.image_topic_name != p.image_topic_name;
  const bool pose_def_changed = runtime_.pose_def_name != p.pose_def_name;
  const bool device_changed = !SameCStr(runtime_.device_name, p.device_name);

  RuntimeParam next = p;
  if (device_changed)
  {
    XR_LOG_WARN(
        "WebotsCamera: device_name runtime change '%s' -> '%s' requires reconstruction. "
        "Keeping current camera binding.",
        SafeCStr(runtime_.device_name), SafeCStr(p.device_name));
    next.device_name = runtime_.device_name;
  }

  runtime_ = std::move(next);
  if (topic_changed)
  {
    image_frame_topic_ = SharedImageTopic(runtime_.image_topic_name.c_str(), image_topic_config);
  }
  if (pose_def_changed)
  {
    cam_node_ = nullptr;
  }
  UpdateParameters();
}

template <CameraTypes::CameraInfo CameraInfoV>
void WebotsCamera<CameraInfoV>::UpdateParameters()
{
  if (!cam_)
  {
    return;
  }

  // 帧率变化会同时影响 Webots 的相机采样周期和我们自己的发布节流桶。
  const int new_period = FpsToPeriodMS(runtime_.fps, sample_period_ms_);
  const int new_publish_interval =
      std::max(1, static_cast<int>(std::lround(static_cast<double>(new_period) /
                                               static_cast<double>(time_step_ms_))));
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

  // 曝光可以直接透传给 Webots 相机。
  cam_->setExposure(runtime_.exposure);
  XR_LOG_INFO("WebotsCamera: exposure=%.3f applied", runtime_.exposure);

  if (runtime_.gain != 0.0)
  {
    XR_LOG_WARN(
        "WebotsCamera: gain=%.3f requested, but not supported by Webots Camera. "
        "Value recorded only.",
        runtime_.gain);
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
void WebotsCamera<CameraInfoV>::SetExposure(double exposure)
{
  // RamFS 或外部配置热更新曝光时，直接复用同一条应用路径。
  runtime_.exposure = exposure;
  if (cam_)
  {
    cam_->setExposure(runtime_.exposure);
    XR_LOG_INFO("SetExposure(): exposure=%.3f", runtime_.exposure);
  }
  else
  {
    XR_LOG_WARN("SetExposure(): camera not ready yet.");
  }
}

template <CameraTypes::CameraInfo CameraInfoV>
void WebotsCamera<CameraInfoV>::SetGain(double gain)
{
  // 当前 Webots Camera 没有真实 gain 接口，这里只保留参数兼容性。
  runtime_.gain = gain;
  XR_LOG_WARN("SetGain(): Webots camera has no native gain. recorded=%.3f",
              runtime_.gain);
}

template <CameraTypes::CameraInfo CameraInfoV>
void WebotsCamera<CameraInfoV>::ThreadFun(WebotsCamera<CameraInfoV>* self)
{
  XR_LOG_INFO("Publishing image!");

  while (self->running_.load())
  {
    if (!self->robot_ || !self->cam_)
    {
      break;
    }

    LibXR::Thread::Sleep(self->time_step_ms_);

    // 只有跨过新的发布桶时才真正取图，避免仿真步长高于目标图像帧率时过发。
    if (!self->ShouldPublishFrame(self->CurrentSimStep()))
    {
      continue;
    }

    const unsigned char* rgba = self->cam_->getImage();
    if (rgba)
    {
      self->PublishImageFrame(rgba, self->CurrentSimTimeUs());
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
