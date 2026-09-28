# WebotsCamera

WebotsCamera 是 Webots 侧的相机/IMU 传感器端点。模块只负责采集和发布原始传感器数据：
IMU 拆成 gyro / accl / quat 三路 topic，相机图像只在触发 GPIO 进入有效电平时提交。

图像触发由 CameraSync 完成；图像与 IMU 的配对由 CameraFrameSync 完成。

## 功能边界

- 每个 Webots step 发布一组原始 IMU 样本。
- 模块自身实现 `LibXR::GPIO`，作为相机触发线交给 CameraSync；CameraSync 写该 GPIO 后
  才提交图像。
- `fps` 只控制 Webots Camera 的底层渲染采样周期，实际图像提交由 CameraSync 的微秒触发
  周期决定。
- 图像写入 `CameraBase::ImageFrame`，像素格式固定为紧密排列 BGR8（Webots BGRA 转 BGR）；
  每帧携带原生几何，通过 `SharedFrame` 保持所有权。
- 采集线程（`webots_camera`，实时优先级）随 LibXR Webots timebase 唤醒，原始采样时间戳取
  `robot->getTime()` 的仿真时间。
- STOP/START/FRAME 协议、序列号、触发回执和图像/IMU 同步由 CameraSync / CameraFrameSync
  处理。
- 模块不做 IMU/图像配对，也不发布同步后的 `ImuStamped`。

## 类型与档位

模板参数 `FrameLayoutV` 为 `CameraTypes::FrameLayout`，必须是紧密排列的 BGR8
（`step == width * 3`，编译期检查）。启动时还检查 Webots Camera 实际宽高与布局一致，
不一致则抛出异常。

标定通过构造参数 `calibration` 独立传入，原生标定尺寸必须等于帧布局。模块固定提供一个
WIDE 档位，几何为原生尺寸、ROI 偏移 0、decimation 1。`Profiles()` 返回稳定只读视图；
`SwitchProfile(WIDE, applied)` 返回该固定档位，其他档位返回 `NOT_SUPPORT`。

## Topic

| Topic | Payload | 时间戳 | 说明 |
| --- | --- | --- | --- |
| `<device_name>_gyro` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | 角速度，单位 rad/s，domain 为 `raw_topic_domain_name`。 |
| `<device_name>_accl` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | 线加速度，单位 m/s^2，domain 为 `raw_topic_domain_name`。 |
| `<device_name>_quat` | `LibXR::Quaternion<float>` | Topic timestamp | 姿态四元数，顺序 wxyz，domain 为 `raw_topic_domain_name`。 |
| `image_topic_name` | `const CameraBase::SharedFrame*` | `ImageFrame::timestamp_us` | 同步回调期间借用；跨回调使用必须复制 `SharedFrame`。 |
| `gimbal_quat`（domain `host`） | `LibXR::Quaternion<float>` | Topic timestamp | 与 `<device_name>_quat` 同源的姿态。 |

默认配置下原始 IMU topic 为 `camera_gyro`、`camera_accl`、`camera_quat`。`imu_topic_name`
只交给 CameraBase 作为同步 IMU topic 名，WebotsCamera 本身不向它发布。

## 时间与触发同步

LibXR Webots timebase 在每次仿真 step 后推进并唤醒采集线程。WebotsCamera 将
`robot->getTime()` 换算为微秒，用于 step 去重和原始 IMU 样本时间戳。

触发 GPIO 进入有效电平时，模块记下最近一次发布的原始 IMU 时间戳作为触发时间（尚无 IMU
样本时用 LibXR timebase 当前时间）。之后第一个时间不早于它的采集 step 读取当前 Webots
图像并提交，图像时间戳为触发时间，不以当前采样时间覆盖。

```text
IMU topic -> CameraSync -> GPIO edge -> WebotsCamera CommitImage()
```

CameraSync 根据微秒时间戳和 `trigger_period_us` 调度触发，处理 STOP/START 命令并发布 FRAME
事件。WebotsCamera 只响应 GPIO 有效边沿，不解析同步命令。CameraFrameSync 根据 CameraSync
回执锁定 IMU 时间轴，再为图像选择对应 IMU；WebotsCamera 不比较图像时间戳和 IMU 时间戳，
也不发布同步结果。

## 坐标系

模块对外发布公开本体系 `B`：

- 右手系。
- `x` 向右。
- `y` 向前。
- `z` 向上。

`<device_name>_quat`、`<device_name>_gyro`、`<device_name>_accl` 和 `host` 域的
`gimbal_quat` 均使用这套坐标系。角速度正方向遵循各轴右手定则。

当前 Webots world 的 IMU 安装零位让原始 `+X` roll 对应相机低头，和公开
`B` 系的“`+X` roll 抬头”相反；yaw 轴同号。因此模块在发布 topic 前做固定传感器安装变换：
向量取 `diag(-1, -1, 1)`，姿态四元数按同一变换（绕 `z` 轴 180°）换基。这个变换只属于 Webots 传感器
适配层，下游（如 ArmorTracker）不应再做一次。

## Webots 设备约定

world 中需要一个名为 `device_name` 的 Camera。当 `pose_def_name = camera` 时，还需要：

- `camera_gyro`
- `camera_accelerometer`
- `camera_inertial_unit`

三类 IMU 设备应同刚体安装，缺少任一设备时构造抛出异常。姿态来自
`InertialUnit::getQuaternion()`，Webots 返回的 xyzw 在模块内转换为 wxyz。

## RamFS 命令

CameraBase 以 `device_name` 为名创建命令文件：`set_exposure <值>` 设置 Webots Camera 曝光；
`set_gain <值>` 仅记录警告并忽略（Webots Camera 不支持增益）。

## 依赖

- `QDU-Robomaster/CameraBase`：相机基类、帧布局与共享图像所有权。
- 外部：OpenCV 4（`core`、`imgproc`，用于 BGRA 到 BGR 转换）、Webots C++ API、Eigen。

本模块只能在 LibXR Webots 后端下构建（`-DLIBXR_SYSTEM=webots -DLIBXR_DRIVER=webots`），
它通过该后端提供的 `_libxr_webots_robot_handle` 访问 Webots `Robot`。

## 构造接口

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class WebotsCamera : public CameraBase<FrameLayoutV>, public LibXR::GPIO;

explicit WebotsCamera(
    LibXR::RamFS& ramfs,
    CameraCalibration calibration = DefaultCalibration(),
    RuntimeParam runtime = DefaultRuntime());
```

模板参数：

- `FrameLayoutV`：图像布局，必须是紧密排列 BGR8，且与 Webots Camera 分辨率一致。

依赖：

- `ramfs`：`LibXR::RamFS`，注册相机命令文件。

配置：

- `calibration`：原生标定 `CameraTypes::CameraCalibration`，尺寸必须等于帧布局。
  默认 `DefaultCalibration()` 为 1280x720、`fx = fy = 800`、主点 `(640, 360)`、
  `PLUMB_BOB` 零畸变。
- `runtime`（`RuntimeParam`）：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device_name` | `"camera"` | Webots Camera 设备名、原始 IMU topic 前缀和 RamFS 命令文件名。 |
| `fps` | `30` | 底层渲染采样频率，量化为整数个 Webots step；必须为正；不是触发发布频率。 |
| `exposure` | `1.0` | Webots Camera 曝光值。 |
| `gain` | `0.0` | 保留参数；Webots Camera 不支持 gain，非零值会被忽略。 |
| `pose_def_name` | `"camera"` | IMU 设备名前缀。 |
| `image_topic_name` | `"camera_image"` | 原始图像 topic 名。 |
| `imu_topic_name` | `"camera_imu"` | 同步 IMU topic 名，传给 CameraBase。 |
| `raw_topic_domain_name` | `"mcu"` | 原始 IMU topic domain；进程内调试可改为默认域 `"libxr_def_domain"`。 |
| `trigger_active_level` | `true` | 触发有效电平，进入该电平的边沿提交图像。 |
| `trigger_period_us` | `20000` | 固定 WIDE 档位声明的触发周期，单位 us；应与 CameraSync 配置相同，且必须非零。 |

名称字段不能为空；参数非法时构造抛出 `std::invalid_argument`。

## 使用

```sh
xrobot module add QDU-Robomaster/WebotsCamera
xrobot setup
xrobot instance add QDU-Robomaster/WebotsCamera
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `ramfs` 填为 BSP 中用 `XR_REGISTER` 注册的 RamFS 对象名。帧布局用 constexpr 定义，必须与
Webots Camera 的实际输出一致；默认标定是 1280x720，因此这里使用 1280x720 布局：

```yaml
constexpr_includes:
  - CameraBase.hpp
constexprs:
  FrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 1280, .height = 720, .step = 3840, .encoding = CameraTypes::Encoding::BGR8}'
modules:
  - module: QDU-Robomaster/WebotsCamera
    id: webotscamera_0
    template_args:
      - ProjectConstexpr::FrameLayout
    args:
      - ramfs: ramfs
      - calibration: WebotsCamera<ProjectConstexpr::FrameLayout>::DefaultCalibration()
      - runtime: WebotsCamera<ProjectConstexpr::FrameLayout>::DefaultRuntime()
```

使用其他分辨率时，把 `calibration` 换成与布局同尺寸的标定（可同样定义为
`CameraTypes::CameraCalibration` 类型的 constexpr）。`runtime` 也可以写成 YAML map，
字符串字段写成 C++ 字符串字面量，例如：

```yaml
runtime:
  device_name: '"camera"'
  fps: 100
  exposure: 0.8
  gain: 0.0
  pose_def_name: '"camera"'
  image_topic_name: '"camera_image"'
  imu_topic_name: '"camera_imu"'
  raw_topic_domain_name: '"libxr_def_domain"'
  trigger_active_level: true
  trigger_period_us: 20000
```

BSP 侧：

```cpp
XR_REGISTER(ramfs, LibXR::RamFS);
```

后续的 CameraSync（`camera_pin`）和 CameraFrameSync（`camera`）实例用 id `webotscamera_0`
引用本实例，它们必须列在本实例之后。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/WebotsCamera`
（在 BSP 中）打印当前的构造函数。
