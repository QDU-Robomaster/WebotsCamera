#include "WebotsCamera.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <opencv2/imgproc.hpp>

#include "libxr_system.hpp"
#include "logger.hpp"
#include "thread.hpp"
#include "timer.hpp"
#include "transform.hpp"
#include "webots/Supervisor.hpp"

// === 工具函数 ===============================================================
int WebotsCamera::FpsToPeriodMS(int fps, int fallback_ms)
{
  if (fps <= 0)
  {
    return fallback_ms;
  }
  const double P = 1000.0 / static_cast<double>(fps);
  int ms = static_cast<int>(std::lround(P));
  return ms < 1 ? 1 : ms;
}

// === 构造 / 析构 ============================================================
WebotsCamera::WebotsCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                           CameraBase::CameraInfo info, RuntimeParam runtime)
    // 注意：必须显式构造 CameraBase，让 RamFS 命令入口挂载成功
    : CameraBase(hw, runtime.device_name),
      info_(info),
      runtime_(runtime),
      robot_(_libxr_webots_robot_handle)
{
  XR_LOG_INFO("Starting WebotsCamera!");

  // 1) 获取 Robot 句柄 & basicTimeStep

  if (!robot_)
  {
    XR_LOG_ERROR("Webots robot handle is null!");
    exit(-1);
  }
  time_step_ms_ = static_cast<int>(robot_->getBasicTimeStep());
  if (time_step_ms_ <= 0)
  {
    time_step_ms_ = 16;  // 容错
  }

  // 2) 获取 Camera 设备
  cam_ = robot_->getCamera(runtime_.device_name);
  if (!cam_)
  {
    XR_LOG_ERROR("Webots Camera '%s' not found!", runtime_.device_name);
    throw std::runtime_error("WebotsCamera: camera device not found");
  }

  // 3) 预分配最大缓冲
  frame_buf_ = std::make_unique<std::array<uint8_t, BUF_BYTES>>();

  info_.encoding = CameraBase::Encoding::BGR8;

  // 4) 启用相机（以 fps 近似设置采样周期）
  sample_period_ms_ = FpsToPeriodMS(runtime_.fps, 33);
  publish_interval_steps_ =
      std::max(1, static_cast<int>(std::lround(static_cast<double>(sample_period_ms_) /
                                               static_cast<double>(time_step_ms_))));
  cam_->enable(sample_period_ms_);

  // 5) 初始曝光（若 runtime_ 有传入）
  cam_->setExposure(runtime_.exposure);
  XR_LOG_INFO("Initial exposure set to %.3f", runtime_.exposure);

  // 6) 启动抓取线程
  running_.store(true);
  capture_thread_.Create(this, ThreadFun, "WebotsCameraThread",
                         static_cast<size_t>(1024 * 128),
                         LibXR::Thread::Priority::REALTIME);

  XR_LOG_PASS(
      "Webots camera enabled: name=%s, period=%d ms, world_dt=%d ms, publish_every=%d step(s)",
      runtime_.device_name, sample_period_ms_, time_step_ms_, publish_interval_steps_);

  supervisor_ = hw.FindOrExit<webots::Supervisor>({"supervisor"});

  auto timer_handle = LibXR::Timer::CreateTask<WebotsCamera*>(
      [](WebotsCamera* self)
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

WebotsCamera::~WebotsCamera()
{
  running_.store(false);

  if (cam_)
  {
    cam_->disable();
    cam_ = nullptr;
  }

  XR_LOG_INFO("WebotsCamera destroyed!");
}


CameraBase::PoseStamped WebotsCamera::ReadCameraPoseStamped(
    LibXR::MicrosecondTimestamp timestamp) const
{
  CameraBase::PoseStamped pose{};
  pose.timestamp = timestamp;

  if (cam_node_ == nullptr && supervisor_ != nullptr)
  {
    auto* mutable_self = const_cast<WebotsCamera*>(this);
    mutable_self->cam_node_ = supervisor_->getFromDef("camera");
  }
  if (cam_node_ == nullptr)
  {
    return pose;
  }

  static const LibXR::RotationMatrix<double> COMPENSATION =
      LibXR::RotationMatrix<double>(0, 1, 0, 1, 0, 0, 0, 0, -1);

  const double* r9_cam = cam_node_->getOrientation();
  LibXR::RotationMatrix<double> r_final =
      LibXR::RotationMatrix<double>(r9_cam[0], r9_cam[1], r9_cam[2], r9_cam[3],
                                    r9_cam[4], r9_cam[5], r9_cam[6], r9_cam[7],
                                    r9_cam[8]) *
      COMPENSATION;

  LibXR::Quaternion<double> quat = r_final;
  LibXR::Quaternion<float> quat_f(static_cast<float>(quat.x()),
                                  static_cast<float>(quat.y()),
                                  static_cast<float>(quat.z()),
                                  static_cast<float>(quat.w()));
  LibXR::EulerAngle<float> eulr = quat_f.ToEulerAngle();
  eulr = LibXR::EulerAngle<float>(-eulr.Pitch(), eulr.Yaw(), eulr.Roll());
  pose.rotation = eulr.ToQuaternion();

  const double* p3_cam = cam_node_->getPosition();
  pose.translation = LibXR::Position<float>(static_cast<float>(p3_cam[0]),
                                            static_cast<float>(p3_cam[1]),
                                            static_cast<float>(p3_cam[2]));
  return pose;
}

// === 运行时参数更新 =========================================================
void WebotsCamera::SetRuntimeParam(const RuntimeParam& p)
{
  runtime_ = p;
  UpdateParameters();
}

void WebotsCamera::UpdateParameters()
{
  if (!cam_)
  {
    return;
  }

  // 1) 更新 fps -> enable 周期
  const int NEW_PERIOD = FpsToPeriodMS(runtime_.fps, sample_period_ms_);
  const int NEW_PUBLISH_INTERVAL =
      std::max(1, static_cast<int>(std::lround(static_cast<double>(NEW_PERIOD) /
                                               static_cast<double>(time_step_ms_))));
  if (NEW_PERIOD != sample_period_ms_)
  {
    sample_period_ms_ = NEW_PERIOD;
    cam_->disable();
    cam_->enable(sample_period_ms_);
    XR_LOG_INFO("WebotsCamera: fps target=%d -> period=%d ms", runtime_.fps,
                sample_period_ms_);
  }
  publish_interval_steps_ = NEW_PUBLISH_INTERVAL;
  last_publish_bucket_ = UINT64_MAX;

  // 2) 更新曝光
  cam_->setExposure(runtime_.exposure);
  XR_LOG_INFO("WebotsCamera: exposure=%.3f applied", runtime_.exposure);

  // 3) “增益”仅记录并告警
  if (runtime_.gain != 0.0)
  {
    XR_LOG_WARN(
        "WebotsCamera: gain=%.3f requested, but not supported by Webots Camera. "
        "Value recorded only.",
        runtime_.gain);
  }
}

// === CameraBase 命令接口实现 ================================================
void WebotsCamera::SetExposure(double exposure)
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

void WebotsCamera::SetGain(double gain)
{
  runtime_.gain = gain;
  XR_LOG_WARN("SetGain(): Webots camera has no native gain. recorded=%.3f",
              runtime_.gain);
}

// === 采集线程 ===============================================================
void WebotsCamera::ThreadFun(WebotsCamera* self)
{
  XR_LOG_INFO("Publishing image!");

  while (self->running_.load())
  {
    if (!self->robot_ || !self->cam_)
    {
      break;
    }

    LibXR::Thread::Sleep(self->time_step_ms_);

    const uint64_t sim_time_ms = static_cast<uint64_t>(std::llround(std::max(0.0, self->robot_->getTime()) * 1000.0));
    const uint64_t sim_step =
        self->time_step_ms_ > 0
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

    // 取图：Webots 提供的图像是 BGRA（8UC4），指针在下一次 step 前有效
    const unsigned char* rgba = self->cam_->getImage();
    const int W = self->cam_->getWidth();
    const int H = self->cam_->getHeight();

    if (rgba && W > 0 && H > 0 && W <= MAX_W && H <= MAX_H)
    {
      // 将 BGRA 转为 BGR，避免 detector 再做一次 RGB->BGR 转换。
      cv::Mat src(H, W, CV_8UC4, const_cast<unsigned char*>(rgba));
      cv::Mat dst(H, W, CV_8UC3, self->frame_buf_->data(), static_cast<size_t>(W) * CH);
      cv::cvtColor(src, dst, cv::COLOR_BGRA2BGR);

      // 更新 CameraInfo
      self->info_.width = static_cast<uint32_t>(W);
      self->info_.height = static_cast<uint32_t>(H);
      self->info_.step = static_cast<uint32_t>(W * CH);
      self->info_.timestamp = LibXR::MicrosecondTimestamp(
          static_cast<uint64_t>(std::llround(self->robot_->getTime() * 1000000.0)));
      auto pose_msg = self->ReadCameraPoseStamped(self->info_.timestamp);

      // 先发本帧姿态，再发 camera_info 和图像，确保后级按 image timestamp 对齐。
      self->camera_pose_topic_.Publish(pose_msg);
      self->info_topic_.Publish(self->info_);
      self->frame_topic_.Publish(dst);

      self->fail_count_ = 0;
    }
    else
    {
      self->fail_count_++;
      XR_LOG_WARN("WebotsCamera: getImage failed or invalid size (W=%d H=%d).", W, H);
    }

    if (self->fail_count_ > 5)
    {
      XR_LOG_ERROR("WebotsCamera failed repeatedly (%d)!", self->fail_count_);
    }
  }
}
