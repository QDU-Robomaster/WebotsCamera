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
#include "transform.hpp"

#include <webots/Camera.hpp>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

extern webots::Robot* _libxr_webots_robot_handle;

/**
 * @brief Webots 相机模块。
 *
 * 负责取图、节流发布，以及同步输出相机位姿。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class WebotsCamera : public LibXR::Application, public CameraBase<CameraInfoV>
{
 public:
  using Self = WebotsCamera<CameraInfoV>;
  using Base = CameraBase<CameraInfoV>;
  using CameraInfo = typename Base::CameraInfo;
  using SharedImageFrame = typename Base::SharedImageFrame;
  using SharedImageTopic = LibXR::LinuxSharedTopic<SharedImageFrame>;

  static inline constexpr CameraInfo camera_info = Base::camera_info;
  static constexpr int channel_count = 3;
  static constexpr int frame_width = static_cast<int>(camera_info.width);
  static constexpr int frame_height = static_cast<int>(camera_info.height);
  static constexpr std::size_t frame_step = static_cast<std::size_t>(camera_info.step);
  static constexpr LibXR::LinuxSharedTopicConfig image_topic_config{
      .slot_num = 4,
      .subscriber_num = 2,
      .queue_num = 4,
  };

  static_assert(camera_info.encoding == CameraTypes::Encoding::BGR8,
                "WebotsCamera requires BGR8 output encoding");
  static_assert(frame_step == static_cast<std::size_t>(camera_info.width) * channel_count,
                "WebotsCamera requires packed BGR step");

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
    std::string image_topic_name = "image_frame";
    std::string pose_def_name = "camera";
  };

  explicit WebotsCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                        RuntimeParam runtime)
      : Base(hw, runtime.device_name),
        runtime_(std::move(runtime)),
        image_frame_topic_(runtime_.image_topic_name.c_str(), image_topic_config),
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
    const bool topic_changed = runtime_.image_topic_name != runtime.image_topic_name;
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
    if (topic_changed)
    {
      image_frame_topic_ =
          SharedImageTopic(runtime_.image_topic_name.c_str(), image_topic_config);
    }
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
  static const char* SafeCStr(const char* text) { return text != nullptr ? text : ""; }

  static bool SameCStr(const char* lhs, const char* rhs)
  {
    return std::strcmp(SafeCStr(lhs), SafeCStr(rhs)) == 0;
  }

  static int FpsToPeriodMS(int fps, int fallback_ms)
  {
    if (fps <= 0)
    {
      return fallback_ms;
    }

    const double period = 1000.0 / static_cast<double>(fps);
    const int ms = static_cast<int>(std::lround(period));
    return ms < 1 ? 1 : ms;
  }

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
    sample_period_ms_ = FpsToPeriodMS(runtime_.fps, 33);
    publish_interval_steps_ =
        std::max(1, static_cast<int>(std::lround(static_cast<double>(sample_period_ms_) /
                                                 static_cast<double>(time_step_ms_))));
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

  void RefreshCameraNode()
  {
    if (cam_node_ == nullptr && supervisor_ != nullptr)
    {
      cam_node_ = supervisor_->getFromDef(runtime_.pose_def_name.c_str());
    }
  }

  void PublishPose(PoseStamped pose)
  {
    const LibXR::EulerAngle<float> eulr = pose.rotation.ToEulerAngle();
    XR_LOG_DEBUG("WebotsCamera: camera eulr: %.3f, %.3f, %.3f", eulr.Roll(),
                 eulr.Pitch(), eulr.Yaw());
    camera_pose_topic_.Publish(pose);
    gimbal_rotation_topic_.Publish(pose.rotation);
  }

  PoseStamped ReadCameraPoseStamped(LibXR::MicrosecondTimestamp timestamp)
  {
    PoseStamped pose{};
    pose.timestamp = timestamp;

    RefreshCameraNode();
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

  void PublishImageFrame(const unsigned char* rgba, LibXR::MicrosecondTimestamp timestamp)
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

      const auto timestamp = self->CurrentSimTimeUs();
      auto pose = self->ReadCameraPoseStamped(timestamp);
      self->PublishPose(pose);

      const unsigned char* rgba = self->cam_->getImage();
      if (rgba != nullptr)
      {
        self->PublishImageFrame(rgba, timestamp);
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
