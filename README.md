# WebotsCamera

WebotsCamera 是 Webots 侧的相机/IMU 数据源。模块只负责采集和发布原始传感器数据：相机提交原始图像，IMU 拆成 gyro / accl / quat 三路 topic。

图像与 IMU 的配对由 CameraFrameSync 完成。

## 功能边界

- 每个 Webots step 发布一组原始 IMU 样本。
- 图像按 `fps` 分频发布，实际周期量化到 `basicTimeStep`。
- 图像写入 `CameraBase::ImageFrame`，像素格式固定为紧密排列 BGR8。
- 传感器时间戳来自 libxr Webots timebase，不直接读取 `robot->getTime()`。
- `sensor_sync_cmd` 只影响下一次图像间隔，用于同步探针。
- 模块不做 IMU/图像配对，也不发布同步后的 `ImuStamped`。

## 运行参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device_name` | `camera` | Webots Camera 设备名，也是原始 IMU topic 前缀。 |
| `fps` | `30` | 图像目标发布频率，最终量化为整数个 Webots step。 |
| `exposure` | `1.0` | Webots Camera 曝光值。 |
| `gain` | `0.0` | 保留参数；Webots Camera 不支持 gain，非零值会被忽略。 |
| `pose_def_name` | `camera` | IMU 设备名前缀。 |
| `image_topic_name` | `camera_image` | 原始图像 topic 名。 |
| `imu_topic_name` | `camera_imu` | 同步后 IMU topic 名，传给 CameraBase 供 CameraFrameSync 发布。 |

## Topic

| Topic | Payload | 时间戳 | 说明 |
| --- | --- | --- | --- |
| `<device_name>_gyro` | `std::array<float, 3>` | Topic timestamp | 角速度，单位 rad/s。 |
| `<device_name>_accl` | `std::array<float, 3>` | Topic timestamp | 线加速度，单位 m/s^2。 |
| `<device_name>_quat` | `std::array<float, 4>` | Topic timestamp | 姿态四元数，顺序 wxyz。 |
| `image_topic_name` | `CameraBase::ImageFrame` | `ImageFrame::timestamp_us` | 原始 BGR8 图像。 |
| `sensor_sync_cmd` | `CameraBase::SensorSyncCmd` | 不使用 | 一次性同步探针命令。 |
| `gimbal/rotation` | `LibXR::Quaternion<float>` | Topic timestamp | 与 `<device_name>_quat` 同源。 |

默认配置下原始 IMU topic 为 `camera_gyro`、`camera_accl`、`camera_quat`。

## 时间与同步探针

libxr Webots timebase 在每次仿真 step 后推进。WebotsCamera 使用 `LibXR::Timebase::GetMicroseconds()` 作为传感器时间戳。

模块不调用 `robot->getTime()`。

正常图像间隔为 `N` 个 Webots step。收到 `sensor_sync_cmd` 后，WebotsCamera 把下一张图像延后一个基础图像周期，因此图像间隔表现为：

```text
N -> 2N -> N
```

CameraFrameSync 根据图像传感器时间差识别探针位置，再在 IMU 时间轴中选帧。WebotsCamera 不比较图像时间戳和 IMU 时间戳，也不发布同步结果。

## 坐标系

Webots world 必须把 Camera、Gyro、Accelerometer、InertialUnit 安装成同一套坐标系：

- 右手系。
- `x` 向右。
- `y` 向前。
- `z` 向上。

`camera_quat`、`camera_gyro`、`camera_accl`、`gimbal/rotation` 均使用这套坐标系。角速度正方向遵循各轴右手定则。

模块不做运行时零位标定，也不补偿 world 中的安装误差；坐标错误应在 world 中修正。

## Webots 设备约定

当 `pose_def_name = camera` 时，world 中需要提供：

- `camera_gyro`
- `camera_accelerometer`
- `camera_inertial_unit`

三类 IMU 设备应与 Camera 节点共用同一坐标定义。`camera_quat` 和 `gimbal/rotation` 均来自 `InertialUnit::getQuaternion()`。

Webots 返回的 xyzw 会在模块内转换为 wxyz。

## 依赖

- `qdu-future/CameraBase`
