#include "WebotsCamera.hpp"

#include <cstdlib>
#include <opencv2/imgproc.hpp>

#include "logger.hpp"

// === 工具函数 ===============================================================
int WebotsCamera::fps_to_period_ms(int fps, int fallback_ms)
{
  if (fps <= 0) return fallback_ms;
  const double p = 1000.0 / static_cast<double>(fps);
  int ms = static_cast<int>(p + 0.5);
  return ms < 1 ? 1 : ms;
}

// === 构造 / 析构 ============================================================
WebotsCamera::WebotsCamera(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                           const CameraBase::CameraInfo info, const RuntimeParam runtime)
    // 注意：必须显式构造 CameraBase，让 RamFS 命令入口挂载成功
    : CameraBase(hw, runtime.device_name.c_str()), info_(info), runtime_(runtime)
{
  XR_LOG_INFO("Starting WebotsCamera!");

  // 1) 获取 Robot 句柄 & basicTimeStep
  robot_ = _libxr_webots_robot_handle;
  if (!robot_)
  {
    XR_LOG_ERROR("Webots robot handle is null!");
    exit(-1);
  }
  time_step_ms_ = static_cast<int>(robot_->getBasicTimeStep());
  if (time_step_ms_ <= 0) time_step_ms_ = 16;  // 容错

  // 2) 获取 Camera 设备
  cam_ = robot_->getCamera(runtime_.device_name);
  if (!cam_)
  {
    XR_LOG_ERROR("Webots Camera '%s' not found!", runtime_.device_name.c_str());
    throw std::runtime_error("WebotsCamera: camera device not found");
  }

  // 3) 预分配最大缓冲
  frame_buf_ = std::make_unique<std::array<uint8_t, BUF_BYTES>>();

  // 4) 启用相机（以 fps 近似设置采样周期）
  sample_period_ms_ = fps_to_period_ms(runtime_.fps, 33);
  cam_->enable(sample_period_ms_);

  // 5) 初始曝光（若 runtime_ 有传入）
  cam_->setExposure(runtime_.exposure);
  XR_LOG_INFO("Initial exposure set to %.3f", runtime_.exposure);

  // 6) 启动抓取线程
  running_.store(true);
  capture_thread_.Create(this, ThreadFun, "WebotsCameraThread",
                         static_cast<size_t>(1024 * 128),
                         LibXR::Thread::Priority::REALTIME);

  XR_LOG_PASS("Webots camera enabled: name=%s, period=%d ms, world_dt=%d ms",
              runtime_.device_name.c_str(), sample_period_ms_, time_step_ms_);

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

// === 运行时参数更新 =========================================================
void WebotsCamera::SetRuntimeParam(const RuntimeParam& p)
{
  runtime_ = p;
  UpdateParameters();
}

void WebotsCamera::UpdateParameters()
{
  if (!cam_) return;

  // 1) 更新 fps -> enable 周期
  const int new_period = fps_to_period_ms(runtime_.fps, sample_period_ms_);
  if (new_period != sample_period_ms_)
  {
    sample_period_ms_ = new_period;
    cam_->disable();
    cam_->enable(sample_period_ms_);
    XR_LOG_INFO("WebotsCamera: fps target=%d -> period=%d ms", runtime_.fps,
                sample_period_ms_);
  }

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

  // 使相机至少“热身”一帧
  if (self->robot_) self->robot_->step(self->time_step_ms_);

  while (self->running_.load())
  {
    if (!self->robot_ || !self->cam_) break;

    // 推进仿真：注意这是“仿真时间步长”，而非采样周期
    int rc = self->robot_->step(self->time_step_ms_);
    if (rc == -1)
    {
      XR_LOG_WARN("Webots simulation ended.");
      break;
    }

    // 取图：Webots 提供的图像是 BGRA（8UC4），指针在下一次 step 前有效
    const unsigned char* rgba = self->cam_->getImage();
    const int W = self->cam_->getWidth();
    const int H = self->cam_->getHeight();

    if (rgba && W > 0 && H > 0 && W <= MAX_W && H <= MAX_H)
    {
      // 将 BGRA 转为 RGB，写入预分配缓冲
      cv::Mat src(H, W, CV_8UC4, const_cast<unsigned char*>(rgba));
      cv::Mat dst(H, W, CV_8UC3, self->frame_buf_->data(), static_cast<size_t>(W) * CH);
      cv::cvtColor(src, dst, cv::COLOR_BGRA2RGB);

      // 更新 CameraInfo
      self->info_.width = static_cast<uint32_t>(W);
      self->info_.height = static_cast<uint32_t>(H);
      self->info_.step = static_cast<uint32_t>(W * CH);
      self->info_.timestamp = LibXR::Timebase::GetMicroseconds();

      // 发布（与其它相机一致：传递 cv::Mat 视图）
      self->frame_topic_.Publish(dst);
      self->info_topic_.Publish(self->info_);

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
