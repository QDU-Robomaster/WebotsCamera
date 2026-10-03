# WebotsCamera

Webots 仿真中的相机与 IMU 传感器端点：发布原始 IMU 数据，并在触发 GPIO 有效时提交图像 / Camera and IMU sensor endpoint in Webots simulation that publishes raw IMU data and commits images when the trigger GPIO becomes active

## 1. 模块作用 / Purpose

WebotsCamera 是 Webots 侧的相机与 IMU 传感器端点。每个 Webots step 发布一组原始 IMU 样本，IMU 拆成 gyro / accl / quat 三路 Topic；相机图像在触发 GPIO 进入有效电平时提交。模块自身实现 `LibXR::GPIO`，作为相机触发线交给 CameraSync，CameraSync 写该 GPIO 后图像才被提交。图像触发由 CameraSync 完成，图像与 IMU 的配对由 CameraFrameSync 完成。

- `fps` 控制 Webots Camera 底层的渲染采样周期，实际图像提交由 CameraSync 的微秒触发周期决定。
- 图像写入 `CameraBase::ImageFrame`，像素格式为紧密排列的 BGR8（Webots BGRA 转 BGR）；每帧携带原生几何，通过 `SharedFrame` 保持所有权。
- 采集线程 `webots_camera`（实时优先级）随 LibXR Webots timebase 唤醒，原始采样时间戳取 `robot->getTime()` 的仿真时间。
- STOP / START / FRAME 协议、序列号、触发回执和图像与 IMU 的同步由 CameraSync 和 CameraFrameSync 处理。
- CameraBase 以 `device_name` 为名创建 RamFS 命令文件：`set_exposure <值>` 设置 Webots Camera 的曝光；`set_gain <值>` 记录警告，增益保持 0（Webots Camera 没有增益控制）。

WebotsCamera is the camera and IMU sensor endpoint on the Webots side. In every Webots step it publishes one raw IMU sample, split into the three Topics gyro / accl / quat; a camera image is committed when the trigger GPIO enters its active level. The Module implements `LibXR::GPIO` itself and hands it to CameraSync as the camera trigger line; an image is committed after CameraSync writes this GPIO. CameraSync performs the image triggering, and CameraFrameSync pairs the images with the IMU data.

- `fps` controls the underlying rendering sampling period of the Webots Camera; the actual image commit is determined by the microsecond trigger period of CameraSync.
- Images are written into `CameraBase::ImageFrame` in packed BGR8 (Webots BGRA converted to BGR); every frame carries its native geometry and keeps ownership through `SharedFrame`.
- The capture thread `webots_camera` (real-time priority) wakes with the LibXR Webots timebase, and the raw sample timestamp is the simulation time of `robot->getTime()`.
- The STOP / START / FRAME protocol, the sequence numbers, the trigger acknowledgements and the image and IMU synchronization are handled by CameraSync and CameraFrameSync.
- CameraBase creates a RamFS command file named after `device_name`: `set_exposure <value>` sets the Webots Camera exposure; `set_gain <value>` logs a warning and the gain stays 0 (the Webots Camera has no gain control).

## 2. 类型与档位 / Types and Profiles

模板参数 `FrameLayoutV` 为 `CameraTypes::FrameLayout`，是紧密排列的 BGR8（`step == width * 3`，编译期检查）。启动时检查 Webots Camera 的实际宽高与布局一致，不一致时抛出异常。

标定通过构造参数 `calibration` 传入，原生标定尺寸等于帧布局。模块提供一个固定的 WIDE 档位，几何为原生尺寸、ROI 偏移 0、decimation 1。`Profiles()` 返回稳定的只读视图；`SwitchProfile(WIDE, applied)` 返回该档位，其他档位返回 `NOT_SUPPORT`。

The template parameter `FrameLayoutV` is a `CameraTypes::FrameLayout` with packed BGR8 (`step == width * 3`, checked at compile time). At startup the actual width and height of the Webots Camera are checked against the layout, and an exception is thrown on a mismatch.

The calibration is passed through the constructor parameter `calibration`, and its native size equals the frame layout. The Module provides one fixed WIDE profile with the native size, ROI offset 0 and decimation 1. `Profiles()` returns a stable read-only view; `SwitchProfile(WIDE, applied)` returns that profile and other profiles return `NOT_SUPPORT`.

## 3. 时间与触发同步 / Timing and Trigger Synchronization

LibXR Webots timebase 在每次仿真 step 后推进并唤醒采集线程。WebotsCamera 把 `robot->getTime()` 换算为微秒，用于 step 去重和原始 IMU 样本的时间戳。

触发 GPIO 进入有效电平时，模块记下最近一次发布的原始 IMU 时间戳作为触发时间（尚无 IMU 样本时取 LibXR timebase 当前时间）。之后第一个时间不早于它的采集 step 读取当前 Webots 图像并提交，图像时间戳为触发时间。

```text
IMU topic -> CameraSync -> GPIO edge -> WebotsCamera CommitImage()
```

CameraSync 根据微秒时间戳和 `trigger_period_us` 调度触发，处理 STOP / START 命令并发布 FRAME 事件；WebotsCamera 响应 GPIO 的有效边沿。CameraFrameSync 根据 CameraSync 回执锁定 IMU 时间轴，再为图像选择对应的 IMU。

LibXR Webots timebase advances after every simulation step and wakes the capture thread. WebotsCamera converts `robot->getTime()` to microseconds for step de-duplication and the raw IMU sample timestamps.

When the trigger GPIO enters its active level, the Module records the timestamp of the latest published raw IMU sample as the trigger time (the current LibXR timebase time when no IMU sample exists yet). The first capture step whose time is not earlier than that reads the current Webots image and commits it, with the trigger time as the image timestamp.

```text
IMU topic -> CameraSync -> GPIO edge -> WebotsCamera CommitImage()
```

CameraSync schedules the triggers from the microsecond timestamps and `trigger_period_us`, handles the STOP / START commands and publishes the FRAME events; WebotsCamera reacts to the active GPIO edge. CameraFrameSync locks the IMU timeline from the CameraSync acknowledgements and then selects the matching IMU data for each image.

## 4. 坐标系 / Coordinate Frame

模块对外发布的公开本体系 `B` 为右手系：`x` 向右，`y` 向前，`z` 向上。`<device_name>_quat`、`<device_name>_gyro`、`<device_name>_accl` 和 `host` 域的 `gimbal_quat` 使用这套坐标系，角速度正方向遵循各轴右手定则。

Webots world 中 IMU 的安装零位使原始 `+X` roll 对应相机低头，与公开 `B` 系的“`+X` roll 抬头”相反，yaw 轴同号。因此模块在发布 Topic 之前做固定的传感器安装变换：向量取 `diag(-1, -1, 1)`，姿态四元数按同一变换（绕 `z` 轴 180°）换基。该变换属于 Webots 传感器适配层，下游模块（如 ArmorTracker）直接使用公开 `B` 系数据。

The public body frame `B` published by the Module is right-handed: `x` points right, `y` forward and `z` up. `<device_name>_quat`, `<device_name>_gyro`, `<device_name>_accl` and the `gimbal_quat` of the `host` domain use this frame, and the positive angular velocity direction follows the right-hand rule of each axis.

The IMU mounting zero of the Webots world makes the raw `+X` roll correspond to the camera pitching down, opposite to the "`+X` roll pitches up" of the public `B` frame, with the same sign on the yaw axis. The Module therefore applies a fixed sensor mounting transform before publishing: vectors use `diag(-1, -1, 1)` and the attitude quaternion changes basis with the same transform (a 180° rotation about the `z` axis). The transform belongs to the Webots sensor adaptation layer, and downstream Modules (such as ArmorTracker) use the public `B` frame data directly.

## 5. 构造接口 / Constructor

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class WebotsCamera : public CameraBase<FrameLayoutV>, public LibXR::GPIO;

explicit WebotsCamera(LibXR::RamFS& ramfs,
                      CameraCalibration calibration = DefaultCalibration(),
                      RuntimeParam runtime = DefaultRuntime());
```

模板参数：

- `FrameLayoutV`：图像布局，紧密排列的 BGR8，与 Webots Camera 分辨率一致。

依赖：

- `ramfs`：`LibXR::RamFS`，注册相机命令文件，取自 BSP 的硬件注册（`XR_REGISTER`）。

配置参数：

- `calibration`：原生标定 `CameraTypes::CameraCalibration`，尺寸等于帧布局。默认 `DefaultCalibration()` 为 1280x720、`fx = fy = 800`、主点 `(640, 360)`、`PLUMB_BOB` 零畸变。
- `runtime`（`RuntimeParam`）：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device_name` | `"camera"` | Webots Camera 设备名、原始 IMU Topic 前缀和 RamFS 命令文件名 |
| `fps` | `30` | 底层渲染采样频率，量化为整数个 Webots step，取正值 |
| `exposure` | `1.0` | Webots Camera 曝光值 |
| `gain` | `0.0` | 增益；Webots Camera 没有增益控制，非零值记录警告后清零 |
| `pose_def_name` | `"camera"` | IMU 设备名前缀 |
| `image_topic_name` | `"camera_image"` | 原始图像 Topic 名 |
| `imu_topic_name` | `"camera_imu"` | 同步 IMU Topic 名，传给 CameraBase |
| `raw_topic_domain_name` | `"mcu"` | 原始 IMU Topic 的 domain；进程内调试可改为默认域 `"libxr_def_domain"` |
| `trigger_active_level` | `true` | 触发有效电平，进入该电平的边沿提交图像 |
| `trigger_period_us` | `20000` | 固定 WIDE 档位声明的触发周期，单位 us，与 CameraSync 配置相同，取非零值 |

名称字段取非空字符串；参数不合法时构造抛出 `std::invalid_argument`。

Template parameter:

- `FrameLayoutV`: the image layout, packed BGR8, equal to the Webots Camera resolution.

Dependencies:

- `ramfs`: the `LibXR::RamFS` that registers the camera command file, taken from the BSP's Registration (`XR_REGISTER`).

Configuration parameters:

- `calibration`: native calibration `CameraTypes::CameraCalibration` whose size equals the frame layout. The default `DefaultCalibration()` is 1280x720, `fx = fy = 800`, principal point `(640, 360)` and zero `PLUMB_BOB` distortion.
- `runtime` (`RuntimeParam`):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `device_name` | `"camera"` | Webots Camera device name, raw IMU Topic prefix and RamFS command file name |
| `fps` | `30` | Underlying rendering sampling rate, quantized to a whole number of Webots steps, positive |
| `exposure` | `1.0` | Webots Camera exposure value |
| `gain` | `0.0` | Gain; the Webots Camera has no gain control, and a non-zero value logs a warning and is reset to zero |
| `pose_def_name` | `"camera"` | IMU device name prefix |
| `image_topic_name` | `"camera_image"` | Raw image Topic name |
| `imu_topic_name` | `"camera_imu"` | Synchronized IMU Topic name, passed to CameraBase |
| `raw_topic_domain_name` | `"mcu"` | Domain of the raw IMU Topics; `"libxr_def_domain"` can be used for in-process debugging |
| `trigger_active_level` | `true` | Active trigger level; the edge entering this level commits an image |
| `trigger_period_us` | `20000` | Trigger period declared by the fixed WIDE profile in us, equal to the CameraSync setting, non-zero |

The name fields are non-empty strings; the constructor throws `std::invalid_argument` for invalid parameters.

## 6. Topic

| Topic | Payload | 时间戳 | 说明 |
| --- | --- | --- | --- |
| `<device_name>_gyro` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | 角速度，单位 rad/s，domain 为 `raw_topic_domain_name` |
| `<device_name>_accl` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | 线加速度，单位 m/s^2，domain 为 `raw_topic_domain_name` |
| `<device_name>_quat` | `LibXR::Quaternion<float>` | Topic timestamp | 姿态四元数，顺序 wxyz，domain 为 `raw_topic_domain_name` |
| `image_topic_name` | `const CameraBase::SharedFrame*` | `ImageFrame::timestamp_us` | 同步回调期间借用，跨回调使用时复制 `SharedFrame` |
| `gimbal_quat`（domain `host`） | `LibXR::Quaternion<float>` | Topic timestamp | 与 `<device_name>_quat` 同源的姿态 |

默认配置下原始 IMU Topic 为 `camera_gyro`、`camera_accl`、`camera_quat`。

| Topic | Payload | Timestamp | Meaning |
| --- | --- | --- | --- |
| `<device_name>_gyro` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | Angular velocity in rad/s, domain `raw_topic_domain_name` |
| `<device_name>_accl` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | Linear acceleration in m/s^2, domain `raw_topic_domain_name` |
| `<device_name>_quat` | `LibXR::Quaternion<float>` | Topic timestamp | Attitude quaternion in wxyz order, domain `raw_topic_domain_name` |
| `image_topic_name` | `const CameraBase::SharedFrame*` | `ImageFrame::timestamp_us` | Borrowed during the synchronous callback; the `SharedFrame` is copied when used across callbacks |
| `gimbal_quat` (domain `host`) | `LibXR::Quaternion<float>` | Topic timestamp | Attitude from the same source as `<device_name>_quat` |

With the default configuration the raw IMU Topics are `camera_gyro`, `camera_accl` and `camera_quat`.

## 7. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/WebotsCamera` 写入含空 `template_args` 的实例。`template_args` 填写为 `constexprs` 中定义的帧布局（与 Webots Camera 的实际输出一致），`calibration` 填写为同尺寸的标定，`ramfs` 填写为 BSP 中注册的 RamFS 名称，`runtime` 写成 YAML map，字符串字段写成 C++ 字符串字面量。以下取自 `bsp-webots-autoaim` 的配置：

`xrobot instance add QDU-Robomaster/WebotsCamera` writes an instance with an empty `template_args`. `template_args` is set to a frame layout defined in `constexprs` (equal to the actual output of the Webots Camera), `calibration` to a calibration of the same size, `ramfs` to a RamFS name registered by the BSP, and `runtime` is written as a YAML map with string fields as C++ string literals. The following is taken from the `bsp-webots-autoaim` configuration:

```yaml
constexpr_namespace: AutoAimRunConfig
constexpr_includes:
  - CameraBase.hpp
constexprs:
  MainFrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 800, .height = 600, .step = 2400, .encoding = CameraTypes::Encoding::BGR8}'
  MainCameraCalibration:
    type: CameraTypes::CameraCalibration
    value: '{.native_width = 800, .native_height = 600, .camera_matrix = {1300.258730617794, 0.0, 400.0, 0.0, 1300.258730617794, 300.0, 0.0, 0.0, 1.0}, .distortion_model = CameraTypes::DistortionModel::PLUMB_BOB, .distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0}, .rectification_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, .projection_matrix = {1300.258730617794, 0.0, 400.0, 0.0, 0.0, 1300.258730617794, 300.0, 0.0, 0.0, 0.0, 1.0, 0.0}}'
  MainImageTopicName:
    type: const char*
    value: "camera_image"
  MainImuTopicName:
    type: const char*
    value: "camera_imu"
modules:
  - module: QDU-Robomaster/WebotsCamera
    id: WebotsCamera_0
    template_args:
      - AutoAimRunConfig::MainFrameLayout
    args:
      - ramfs: ramfs
      - calibration: AutoAimRunConfig::MainCameraCalibration
      - runtime:
          device_name: "camera"
          fps: 100
          exposure: 0.8
          gain: 0.0
          pose_def_name: "camera"
          image_topic_name: AutoAimRunConfig::MainImageTopicName
          imu_topic_name: AutoAimRunConfig::MainImuTopicName
          raw_topic_domain_name: "libxr_def_domain"
          trigger_active_level: true
          trigger_period_us: 20000
```

CameraSync（`camera_pin`）和 CameraFrameSync（`camera`）的实例以本实例的 id 引用它，列在本实例之后。

The CameraSync (`camera_pin`) and CameraFrameSync (`camera`) instances reference this instance by its id and are listed after it.

## 8. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/CameraBase`：相机基类、帧布局与共享图像所有权。
- OpenCV 4（`core`、`imgproc`，用于 BGRA 到 BGR 的转换）、Webots C++ API、Eigen。
- LibXR，构建时使用 LibXR 的 Webots 后端（`-DLIBXR_SYSTEM=webots -DLIBXR_DRIVER=webots`），模块通过该后端提供的 `_libxr_webots_robot_handle` 访问 Webots `Robot`。

Webots 设备：world 中名为 `device_name` 的 Camera。当 `pose_def_name = camera` 时还需要 `camera_gyro`、`camera_accelerometer` 和 `camera_inertial_unit`，三类 IMU 设备同刚体安装，缺少任一设备时构造抛出异常。姿态来自 `InertialUnit::getQuaternion()`，Webots 返回的 xyzw 在模块内转换为 wxyz。

Dependencies:

- `QDU-Robomaster/CameraBase`: camera base class, frame layout and shared image ownership.
- OpenCV 4 (`core`, `imgproc`, used for the BGRA to BGR conversion), the Webots C++ API and Eigen.
- LibXR, built with the LibXR Webots backend (`-DLIBXR_SYSTEM=webots -DLIBXR_DRIVER=webots`); the Module reaches the Webots `Robot` through the `_libxr_webots_robot_handle` provided by that backend.

Webots devices: a Camera named `device_name` in the world. With `pose_def_name = camera`, `camera_gyro`, `camera_accelerometer` and `camera_inertial_unit` are also required; the three IMU devices are mounted on the same rigid body, and the constructor throws an exception when any of them is missing. The attitude comes from `InertialUnit::getQuaternion()`, and the xyzw returned by Webots is converted to wxyz inside the Module.
