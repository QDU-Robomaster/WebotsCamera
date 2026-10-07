#pragma once

#include <cstdint>
#include <span>

#include "CameraTypes.hpp"

/// 仿真图像到真车帧的换算，不依赖 Webots / Turning a simulated image into a robot
/// frame, without Webots.
namespace WebotsBayer
{
/**
 * @brief 按几何从原生 BGRA 图里取窗，并按 RGGB 抽样成 640×512 BayerRG8。
 *        Cut the window given by the geometry out of a native BGRA image and sample it
 *        into a 640×512 BayerRG8 frame with the RGGB pattern.
 *
 * 跳采 2 与真车相同：每个 4×4 原生块取左上 2×2，帧像素 x 对应原生
 * `roi_x + 4·⌊x/2⌋ + x mod 2`；跳采 1 为 1:1 裁剪。帧像素 (x, y) 的颜色由 x、y
 * 的奇偶决定：偶偶为 R，奇奇为 B，其余为 G。
 *
 * Decimation 2 matches the robot camera: the top-left 2×2 of every 4×4 native block,
 * frame pixel x maps to native `roi_x + 4·⌊x/2⌋ + x mod 2`; decimation 1 is a 1:1
 * crop. The colour of frame pixel (x, y) follows the parities: even-even R, odd-odd B,
 * else G.
 *
 * @param bgra 原生图像，每像素 B、G、R、A 四字节 / Native image, 4 bytes per pixel
 * @param native_width 原生宽度 / Native width
 * @param geometry 帧几何，须在传感器内 / Frame geometry inside the sensor
 * @param bayer 输出，FRAME_BYTES 字节 / Output, FRAME_BYTES bytes
 */
inline void Render(const uint8_t* bgra, uint32_t native_width,
                   const CameraTypes::FrameGeometry& geometry, std::span<uint8_t> bayer)
{
  constexpr uint32_t BLUE = 0;
  constexpr uint32_t GREEN = 1;
  constexpr uint32_t RED = 2;
  const auto native = [&geometry](uint32_t frame, uint32_t roi)
  { return geometry.decimation == 2 ? roi + 4 * (frame / 2) + frame % 2 : roi + frame; };
  for (uint32_t y = 0; y < CameraTypes::FRAME_HEIGHT; ++y)
  {
    const uint8_t* row =
        bgra + static_cast<std::size_t>(native(y, geometry.roi_y)) * native_width * 4;
    uint8_t* out = bayer.data() + static_cast<std::size_t>(y) * CameraTypes::FRAME_WIDTH;
    for (uint32_t x = 0; x < CameraTypes::FRAME_WIDTH; ++x)
    {
      const uint32_t channel =
          y % 2 == 0 ? (x % 2 == 0 ? RED : GREEN) : (x % 2 == 0 ? GREEN : BLUE);
      out[x] = row[static_cast<std::size_t>(native(x, geometry.roi_x)) * 4 + channel];
    }
  }
}
}  // namespace WebotsBayer
