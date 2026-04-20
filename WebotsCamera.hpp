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
#include <stdexcept>

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

template <CameraTypes::CameraInfo CameraInfoV>
class WebotsCamera : public LibXR::Application, public CameraBase<CameraInfoV>
{
 public:
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using SharedImageFrame = typename Base::SharedImageFrame;
  using SharedImageTopic = LibXR::LinuxSharedTopic<SharedImageFrame>;

  static inline constexpr CameraInfo camera_info = Base::camera_info;
  static constexpr int channel_count = 3;
  static constexpr LibXR::LinuxSharedTopicConfig image_topic_config{
      .slot_num = 4,
      .subscriber_num = 2,
      .queue_num = 4,
  };

  struct PoseStamped
  {
    LibXR::MicrosecondTimestamp timestamp{};
    LibXR::Quaternion<float> rotation{};
    LibXR::Position<float> translation{};
  };

  struct RuntimeParam
  {
    const char* device_name = "camera";
    int fps = 30;
    double exposure = 1.0;
    double gain = 0.0;
    const char* image_topic_name = "image_frame";
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
  PoseStamped ReadCameraPoseStamped(LibXR::MicrosecondTimestamp timestamp) const;

 private:
  RuntimeParam runtime_{};
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
WebotsCamera<CameraInfoV>::WebotsCamera(LibXR::HardwareContainer& hw,
                                        LibXR::ApplicationManager& app,
                                        RuntimeParam runtime)
    : Base(hw, runtime.device_name),
      runtime_(runtime),
      image_frame_topic_(runtime_.image_topic_name, image_topic_config),
      robot_(_libxr_webots_robot_handle)
{
  XR_LOG_INFO("Starting WebotsCamera!");

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

  cam_ = robot_->getCamera(runtime_.device_name);
  if (!cam_)
  {
    XR_LOG_ERROR("Webots Camera '%s' not found!", runtime_.device_name);
    throw std::runtime_error("WebotsCamera: camera device not found");
  }

  const uint32_t actual_width = static_cast<uint32_t>(cam_->getWidth());
  const uint32_t actual_height = static_cast<uint32_t>(cam_->getHeight());
  const uint32_t packed_step = actual_width * channel_count;
  if (camera_info.encoding != CameraTypes::Encoding::BGR8)
  {
    throw std::runtime_error("WebotsCamera: constexpr encoding must be BGR8");
  }
  if (camera_info.width != actual_width || camera_info.height != actual_height ||
      camera_info.step != packed_step)
  {
    XR_LOG_ERROR(
        "WebotsCamera: constexpr geometry mismatch width=%u/%u height=%u/%u step=%u/%u",
        camera_info.width, actual_width, camera_info.height, actual_height,
        camera_info.step, packed_step);
    throw std::runtime_error("WebotsCamera: constexpr geometry mismatch");
  }

  sample_period_ms_ = FpsToPeriodMS(runtime_.fps, 33);
  publish_interval_steps_ =
      std::max(1, static_cast<int>(std::lround(static_cast<double>(sample_period_ms_) /
                                               static_cast<double>(time_step_ms_))));
  cam_->enable(sample_period_ms_);

  cam_->setExposure(runtime_.exposure);
  XR_LOG_INFO("Initial exposure set to %.3f", runtime_.exposure);

  running_.store(true);
  capture_thread_.Create(this, ThreadFun, "WebotsCameraThread",
                         static_cast<size_t>(1024 * 128),
                         LibXR::Thread::Priority::REALTIME);

  XR_LOG_PASS(
      "Webots camera enabled: name=%s, period=%d ms, world_dt=%d ms, publish_every=%d step(s)",
      runtime_.device_name, sample_period_ms_, time_step_ms_, publish_interval_steps_);

  supervisor_ = hw.FindOrExit<webots::Supervisor>({"supervisor"});

  auto timer_handle = LibXR::Timer::CreateTask<WebotsCamera<CameraInfoV>*>(
      [](WebotsCamera<CameraInfoV>* self)
      {
        if (self->cam_node_ == nullptr)
        {
          self->cam_node_ = self->supervisor_->getFromDef("camera");
          return;
        }

        auto pose = self->ReadCameraPoseStamped(
            LibXR::MicrosecondTimestamp(static_cast<uint64_t>(std::llround(
                std::max(0.0, self->robot_->getTime()) * 1000000.0))));
        LibXR::EulerAngle<float> eulr = pose.rotation.ToEulerAngle();
        XR_LOG_DEBUG("WebotsCamera: camera eulr: %.3f, %.3f, %.3f", eulr.Roll(),
                     eulr.Pitch(), eulr.Yaw());
        self->gimbal_rotation_topic_.Publish(pose.rotation);
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

  if (cam_node_ == nullptr && supervisor_ != nullptr)
  {
    auto* mutable_self = const_cast<WebotsCamera<CameraInfoV>*>(this);
    mutable_self->cam_node_ = supervisor_->getFromDef("camera");
  }
  if (cam_node_ == nullptr)
  {
    return pose;
  }

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
  runtime_ = p;
  if (topic_changed)
  {
    image_frame_topic_ = SharedImageTopic(runtime_.image_topic_name, image_topic_config);
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

    const uint64_t sim_time_ms = static_cast<uint64_t>(
        std::llround(std::max(0.0, self->robot_->getTime()) * 1000.0));
    const uint64_t sim_step = self->time_step_ms_ > 0
                                  ? (sim_time_ms / static_cast<uint64_t>(self->time_step_ms_))
                                  : sim_time_ms;
    const uint64_t publish_bucket =
        sim_step / static_cast<uint64_t>(self->publish_interval_steps_);
    if (self->last_publish_bucket_ == UINT64_MAX)
    {
      self->last_publish_bucket_ = publish_bucket;
      continue;
    }
    if (publish_bucket == self->last_publish_bucket_)
    {
      continue;
    }
    self->last_publish_bucket_ = publish_bucket;

    const unsigned char* rgba = self->cam_->getImage();
    const int width = self->cam_->getWidth();
    const int height = self->cam_->getHeight();
    const size_t packed_step = static_cast<size_t>(width) * channel_count;
    const size_t data_size = packed_step * static_cast<size_t>(height);

    if (rgba && width == static_cast<int>(camera_info.width) &&
        height == static_cast<int>(camera_info.height) &&
        packed_step == static_cast<size_t>(camera_info.step) &&
        data_size == Base::frame_bytes)
    {
      typename SharedImageTopic::Data shared_image_data;
      const auto create_ans = self->image_frame_topic_.CreateData(shared_image_data);
      if (create_ans != LibXR::ErrorCode::OK)
      {
        self->fail_count_++;
        XR_LOG_WARN("WebotsCamera: shared image slot unavailable (err=%d).",
                    static_cast<int>(create_ans));
        continue;
      }

      SharedImageFrame* shared_frame = shared_image_data.GetData();
      if (shared_frame == nullptr)
      {
        self->fail_count_++;
        XR_LOG_WARN("WebotsCamera: shared image slot returned null payload.");
        continue;
      }

      cv::Mat src(height, width, CV_8UC4, const_cast<unsigned char*>(rgba));
      cv::Mat dst(height, width, CV_8UC3, shared_frame->data.data(),
                  static_cast<size_t>(camera_info.step));
      cv::cvtColor(src, dst, cv::COLOR_BGRA2BGR);

      const auto timestamp = LibXR::MicrosecondTimestamp(
          static_cast<uint64_t>(std::llround(self->robot_->getTime() * 1000000.0)));
      shared_frame->timestamp_us = static_cast<uint64_t>(timestamp);
      shared_frame->sequence = self->frame_sequence_++;

      auto pose_msg = self->ReadCameraPoseStamped(timestamp);
      self->camera_pose_topic_.Publish(pose_msg);

      const auto publish_ans = self->image_frame_topic_.Publish(shared_image_data);
      if (publish_ans != LibXR::ErrorCode::OK)
      {
        self->fail_count_++;
        XR_LOG_WARN("WebotsCamera: shared image publish failed (err=%d).",
                    static_cast<int>(publish_ans));
        continue;
      }

      self->fail_count_ = 0;
    }
    else
    {
      self->fail_count_++;
      XR_LOG_WARN(
          "WebotsCamera: getImage failed or image geometry mismatched (W=%d H=%d bytes=%zu).",
          width, height, data_size);
    }

    if (self->fail_count_ > 5)
    {
      XR_LOG_ERROR("WebotsCamera failed repeatedly (%d)!", self->fail_count_);
    }
  }
}
