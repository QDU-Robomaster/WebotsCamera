#include <cstdio>

#include "WebotsCamera.hpp"

namespace
{
// 只编译链接、不调用：确认驱动对 Webots 与 LibXR 的调用都能链接。
// Compiled and linked, never called: proves the driver's Webots and LibXR calls link.
[[maybe_unused]] void LinkCheck()
{
  constexpr CameraTypes::CameraCalibration CALIBRATION{
      1440, 1080, 2328.69, 2328.69, 720.0, 540.0, {0.0, 0.0, 0.0, 0.0, 0.0}};
  WebotsCamera camera(CALIBRATION, {0.5, 0.5}, "link",
                      {1.0, "link_gyro", "link_accl", "link_quat"});
}
}  // namespace

int main()
{
  std::puts("webots_camera_link_test passed");
  return 0;
}
