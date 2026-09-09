# WebotsCamera

WebotsCamera 是 Webots 侧的相机/IMU 传感器端点。模块只负责采集和发布原始传感器数据：IMU 拆成 gyro / accl / quat 三路 topic，相机图像只在触发 GPIO 进入有效电平时提交。

图像触发由 CameraSync 完成；图像与 IMU 的配对由 CameraFrameSync 完成。

## 功能边界

- 每个 Webots step 发布一组原始 IMU 样本。
- 注册一个 `LibXR::GPIO` 作为相机触发线，CameraSync 写该 GPIO 后才提交图像。
- `fps` 只控制 Webots Camera 的底层渲染采样周期，实际图像提交由 CameraSync 的微秒触发周期决定。
- 图像写入 `CameraBase::ImageFrame`，像素格式固定为紧密排列 BGR8；每帧包含原生几何，通过 SharedFrame 保持所有权。
- 采集线程随 libxr Webots timebase 唤醒，原始采样时间戳取 `robot->getTime()` 的仿真时间。
- STOP/START/FRAME 协议、序列号、触发回执和图像/IMU 同步由 CameraSync / CameraFrameSync 处理。
- 模块不做 IMU/图像配对，也不发布同步后的 `ImuStamped`。

## 类型与档位

模板参数为 `CameraTypes::FrameLayout`，标定通过构造参数 `CameraTypes::CameraCalibration` 独立传入。
构造顺序为 `WebotsCamera(hw, app, calibration, runtime)`。原生标定尺寸必须等于帧布局；固定提供
一个 WIDE 档位，几何为原生尺寸、ROI 偏移 0、decimation 1。`Profiles()` 返回稳定只读视图；
`SwitchProfile(WIDE, applied)` 返回该固定档位，其他档位返回 `NOT_SUPPORT`。

## 运行参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device_name` | `camera` | Webots Camera 设备名，也是原始 IMU topic 前缀。 |
| `fps` | `30` | 底层渲染采样频率，量化为整数个 Webots step；不是触发发布频率。 |
| `exposure` | `1.0` | Webots Camera 曝光值。 |
| `gain` | `0.0` | 保留参数；Webots Camera 不支持 gain，非零值会被忽略。 |
| `pose_def_name` | `camera` | IMU 设备名前缀。 |
| `image_topic_name` | `camera_image` | 原始图像 topic 名。 |
| `imu_topic_name` | `camera_imu` | 同步后 IMU topic 名，传给 CameraBase 供 CameraFrameSync 发布。 |
| `raw_topic_domain_name` | `mcu` | 原始 IMU topic domain；detector-only 本进程调试可改为默认域。 |
| `trigger_gpio_name` | `CAMERA` | 注册给 CameraSync 查找的相机触发 GPIO 名称。 |
| `trigger_active_level` | `true` | 触发有效电平，进入该电平的边沿提交图像。 |
| `trigger_period_us` | `20000` | 固定 WIDE 档位声明的触发周期；应与 CameraSync 配置相同，且必须非零。 |

## Topic

| Topic | Payload | 时间戳 | 说明 |
| --- | --- | --- | --- |
| `<device_name>_gyro` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | 角速度，单位 rad/s。 |
| `<device_name>_accl` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | 线加速度，单位 m/s^2。 |
| `<device_name>_quat` | `LibXR::Quaternion<float>` | Topic timestamp | 姿态四元数，顺序 wxyz。 |
| `image_topic_name` | `const CameraBase::SharedFrame*` | `ImageFrame::timestamp_us` | 同步回调期间借用；跨回调使用必须复制 SharedFrame。 |
| `host/gimbal_quat` | `LibXR::Quaternion<float>` | Topic timestamp | C 板同名姿态 topic，数据与 `<device_name>_quat` 同源。 |

默认配置下原始 IMU topic 为 `camera_gyro`、`camera_accl`、`camera_quat`。

## 时间与触发同步

libxr Webots timebase 在每次仿真 step 后推进并唤醒采集线程。WebotsCamera 将
`robot->getTime()` 换算为微秒，用于 step 去重和原始 IMU 样本时间戳。
触发图像另保留 GPIO 有效边沿的时间戳，不以当前采样时间覆盖它。

CameraSync 根据微秒时间戳和 `trigger_period_us` 调度触发，处理当前 STOP/START 命令并发布 FRAME 事件。
WebotsCamera 只响应 GPIO 有效边沿，把该边沿时间戳写入图像；不解析同步命令，也不实现旧版拉长周期探针。

```text
IMU topic -> CameraSync -> GPIO edge -> WebotsCamera CommitImage()
```

CameraFrameSync 根据 CameraSync 回执 topic 的消息时间戳锁定 IMU 时间轴，再为图像选择对应 IMU。WebotsCamera 不比较图像时间戳和 IMU 时间戳，也不发布同步结果。

## 坐标系

模块对外发布公开本体系 `B`：

- 右手系。
- `x` 向右。
- `y` 向前。
- `z` 向上。

`camera_quat`、`camera_gyro`、`camera_accl`、`host/gimbal_quat` 均使用这套坐标系。角速度正方向遵循各轴右手定则。

当前 Webots world 的 IMU 安装零位让原始 `+X` roll 对应相机低头，和公开
`B` 系的“`+X` roll 抬头”相反；yaw 轴同号。因此模块在发布 topic 前做固定传感器安装变换：
`diag(-1, -1, 1)`。这个变换只属于 Webots 传感器适配层，不能再放到
`ArmorTracker` 里。

## Webots 设备约定

当 `pose_def_name = camera` 时，world 中需要提供：

- `camera_gyro`
- `camera_accelerometer`
- `camera_inertial_unit`

三类 IMU 设备应同刚体安装。`camera_quat` 和 `host/gimbal_quat` 均来自
`InertialUnit::getQuaternion()`，再转换为公开 `B` 系。

Webots 返回的 xyzw 会在模块内转换为 wxyz。

## 依赖

- `qdu-future/CameraBase`
