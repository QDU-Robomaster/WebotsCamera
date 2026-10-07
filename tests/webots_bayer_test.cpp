#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "CameraBase.hpp"
#include "WebotsBayer.hpp"

namespace
{
void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

constexpr uint32_t W = 1440;
constexpr uint32_t H = 1080;

// 每个原生像素的 B、G、R 编码自己的坐标，便于反推取到了哪个像素、哪个通道。
// Each native pixel's B, G, R encode its coordinates, so the sampled pixel and channel
// can be recovered.
uint8_t Code(uint32_t x, uint32_t y, uint32_t channel)
{
  return static_cast<uint8_t>((x * 7 + y * 13 + channel * 101) % 251);
}

std::vector<uint8_t> MakeNative()
{
  std::vector<uint8_t> bgra(static_cast<std::size_t>(W) * H * 4);
  for (uint32_t y = 0; y < H; ++y)
  {
    for (uint32_t x = 0; x < W; ++x)
    {
      for (uint32_t c = 0; c < 3; ++c)
      {
        bgra[(static_cast<std::size_t>(y) * W + x) * 4 + c] = Code(x, y, c);
      }
      bgra[(static_cast<std::size_t>(y) * W + x) * 4 + 3] = 255;
    }
  }
  return bgra;
}

/// 帧像素 (x, y) 的 RGGB 通道（BGRA 下标）/ RGGB channel of frame pixel (x, y).
uint32_t Channel(uint32_t x, uint32_t y)
{
  if (x % 2 == 0 && y % 2 == 0) return 2;  // R
  if (x % 2 == 1 && y % 2 == 1) return 0;  // B
  return 1;                                // G
}

void Check(const std::vector<uint8_t>& native, const CameraTypes::FrameGeometry& g)
{
  std::vector<uint8_t> bayer(CameraTypes::FRAME_BYTES);
  WebotsBayer::Render(native.data(), W, g, bayer);
  for (uint32_t y = 0; y < CameraTypes::FRAME_HEIGHT; y += 7)
  {
    for (uint32_t x = 0; x < CameraTypes::FRAME_WIDTH; x += 5)
    {
      // 取到的原生像素与 CameraTypes 的连续映射相差 0.5（跳采 2）或 0（跳采 1）。
      // The sampled native pixel is within 0.5 (decimation 2) or 0 (decimation 1) of
      // the continuous CameraTypes mapping.
      const CameraTypes::Point2d mapped =
          CameraTypes::FrameToNative(g, {static_cast<double>(x), static_cast<double>(y)});
      const double offset = g.decimation == 2 ? (x % 2 == 0 ? 0.5 : -0.5) : 0.0;
      const double offset_y = g.decimation == 2 ? (y % 2 == 0 ? 0.5 : -0.5) : 0.0;
      const auto nx = static_cast<uint32_t>(std::lround(mapped.x + offset));
      const auto ny = static_cast<uint32_t>(std::lround(mapped.y + offset_y));
      Expect(bayer[y * CameraTypes::FRAME_WIDTH + x] == Code(nx, ny, Channel(x, y)),
             "sampled pixel and channel");
    }
  }
}
}  // namespace

int main()
{
  const std::vector<uint8_t> native = MakeNative();
  Check(native, CameraBase::WIDE_GEOMETRY);
  Check(native, {400, 284, 1});
  Check(native, {800, 568, 1});  // NARROW 右下角 / NARROW bottom-right corner
  // WIDE 首行：帧 x = 0..3 取原生 80、81、84、85 / WIDE first row samples native 80, 81,
  // 84, 85 for frame x = 0..3.
  std::vector<uint8_t> bayer(CameraTypes::FRAME_BYTES);
  WebotsBayer::Render(native.data(), W, CameraBase::WIDE_GEOMETRY, bayer);
  Expect(bayer[0] == Code(80, 24, 2) && bayer[1] == Code(81, 24, 1) &&
             bayer[2] == Code(84, 24, 2) && bayer[3] == Code(85, 24, 1),
         "WIDE skips 2x2 Bayer cells");
  std::puts("webots_bayer_test passed");
  return 0;
}
