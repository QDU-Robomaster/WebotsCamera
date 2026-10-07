# WebotsCamera

Webots 仿真相机：按真车几何输出 BayerRG8，发布 IMU，并作为触发 GPIO / Webots camera that outputs BayerRG8 in the robot camera geometry, publishes the IMU and acts as the trigger GPIO

## 1. 模块作用 / Purpose

WebotsCamera 是 CameraBase 在 Webots 里的驱动。世界里的相机按真车传感器的原生分辨率 1440×1080 渲染，每帧在软件里按当前视角取窗、抽样成 640×512 BayerRG8，所以检测器、跟踪器在仿真里看到的帧与真车的帧几何相同，两档视角都能仿真。它同时承担仿真里 MCU 的两项工作：每个仿真步发布 IMU，并作为 CameraSync 写触发电平的 GPIO。

WebotsCamera is the CameraBase driver in Webots. The world camera renders at the robot sensor's native 1440×1080; each frame is windowed for the current view and sampled into 640×512 BayerRG8 in software, so the detector and tracker see frames with the robot's geometry and both views can be simulated. It also plays two MCU roles in simulation: it publishes the IMU every simulation step and is the GPIO that CameraSync drives for the trigger.

## 2. 世界文件 / World File

| 设备 / Device | 名字 / Name | 要求 / Requirement |
| --- | --- | --- |
| Camera | `<name>` | `width 1440`、`height 1080`，`fieldOfView` 与标定焦距一致 |
| Gyro | `<name>_gyro` | — |
| Accelerometer | `<name>_accelerometer` | — |
| InertialUnit | `<name>_inertial_unit` | — |

机器人（Robot 节点）的名字不能与相机名相同，否则 Webots 找不到相机设备。启动时核对世界里的相机与标定：分辨率相同，`fx`、`fy` 与 `(width / 2) / tan(fieldOfView / 2)` 相差不超过 0.5%，主点距图像中心不超过 1 像素；不一致即致命退出。Webots 的 Lens 畸变不能从控制器读出，标定里的畸变系数须由世界文件的 Lens 参数拟合得到，Lens `center` 保持 `0.5 0.5`。

The robot (Robot node) name must differ from the camera name, otherwise Webots does not find the camera device. At start-up the world camera is checked against the calibration: same resolution, `fx` and `fy` within 0.5% of `(width / 2) / tan(fieldOfView / 2)`, principal point within 1 px of the image centre; a mismatch is fatal. The Webots Lens distortion cannot be read by the controller, so the calibration's distortion coefficients are fitted from the world's Lens parameters, with the Lens `center` kept at `0.5 0.5`.

## 3. 成像 / Imaging

`WebotsBayer::Render` 从原生 BGRA 图取窗并抽样：WIDE 与真车相同，每个 4×4 原生块取左上 2×2，帧像素 x 对应原生 `80 + 4·⌊x/2⌋ + x mod 2`；NARROW 为 1:1 裁剪。帧像素的颜色按 RGGB 由坐标奇偶决定。`ApplyView` 与 `ApplyOffset` 只改软件取窗，立即生效；每帧带渲染时的窗口。

`WebotsBayer::Render` windows and samples the native BGRA image: WIDE matches the robot camera, keeping the top-left 2×2 of every 4×4 native block, so frame pixel x maps to native `80 + 4·⌊x/2⌋ + x mod 2`; NARROW is a 1:1 crop. Each frame pixel's colour follows the RGGB pattern by coordinate parity. `ApplyView` and `ApplyOffset` change only the software window and take effect at once; each frame carries the window it was rendered with.

相机只在触发时渲染：触发电平的上升沿之后的第一个仿真步打开 Webots Camera，下一步取图、抽样并关闭相机。帧时间为取图时的仿真时间，比边沿晚一个仿真步，CameraFrameSync 的同步偏移按此配置。帧计数每个边沿加一，切档后从 0 起；切档前边沿的帧被丢弃。

The camera renders only on demand: on the first simulation step after a rising edge of the trigger level the Webots Camera is enabled, and on the next step the image is read, sampled and the camera disabled. The frame time is the simulation time of the read, one step after the edge; CameraFrameSync's sync offset is configured accordingly. The frame counter advances per edge and restarts at 0 after a view switch; a frame of an edge before the switch is dropped.

## 4. IMU 与线程 / IMU and Threads

每个仿真步读 Gyro、Accelerometer、InertialUnit，转成公共机体系（x 右、y 前、z 上，即 Webots 传感器系绕 z 轴转 180°）后，以仿真时间为 Topic 时间戳，发布到配置的三个 IMU Topic。Topic 名与 CameraFrameSync 配置的 MCU IMU Topic 名相同，载荷与 MCU 相同：角速度、加速度为 `Eigen::Matrix<float, 3, 1>`，姿态为 `LibXR::Quaternion<float>`。

Every simulation step the Gyro, Accelerometer and InertialUnit are read, turned into the body frame (x right, y forward, z up, i.e. the Webots sensor frame turned 180° about z) and published with the simulation time as Topic timestamp on the three configured IMU Topics. The Topic names are the MCU IMU Topic names configured in CameraFrameSync, with the MCU payloads: `Eigen::Matrix<float, 3, 1>` for angular velocity and acceleration, `LibXR::Quaternion<float>` for the attitude.

Webots API 只在模块自己的步进线程里调用，该线程是 LibXR 的 REALTIME 线程，与仿真步锁步运行。CameraBase 的采集线程只从步进线程交出的缓冲里取帧。

The Webots API is called only on the Module's own step thread, a LibXR REALTIME thread that runs in lockstep with the simulation. CameraBase's capture thread only takes frames from the buffer the step thread hands over.

## 5. 配置示例 / Configuration Example

```yaml
modules:
  - module: QDU-Robomaster/WebotsCamera
    id: camera
    args:
      - calibration: AutoAimRunConfig::SimCameraCalibration
      - narrow: {u: 0.5, v: 0.5}
      - name: "gimbal"
      - settings:
          exposure: 1.0
          gyro_topic: "gimbal_gyro"
          accl_topic: "gimbal_accl"
          quat_topic: "gimbal_quat"
  - module: QDU-Robomaster/CameraSync
    id: camera_sync
    args:
      - camera_pin: camera
      # ...
```

## 6. 测试 / Tests

- `tests/webots_bayer_test.cpp`：取窗与抽样的原生像素、通道与 CameraTypes 映射一致（WIDE、NARROW、窗口位于角落）。
- `tests/webots_camera_link_test.cpp`：在 LibXR 的 webots 系统下编译链接整个驱动。

- `tests/webots_bayer_test.cpp`: the sampled native pixels and channels agree with the CameraTypes mapping (WIDE, NARROW, a corner window).
- `tests/webots_camera_link_test.cpp`: compiles and links the whole driver on the LibXR webots system.

## 7. 依赖 / Dependencies

CameraBase、LibXR（webots 系统）、Webots R2025a 控制器库。

CameraBase, LibXR (webots system), the Webots R2025a controller libraries.
