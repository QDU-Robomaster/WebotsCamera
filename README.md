# WebotsCamera

WebotsCamera 是 Webots 侧的相机/IMU 传感器端点。模块只负责采集和发布原始传感器数据：IMU 拆成 gyro / accl / quat 三路 topic，相机图像只在触发 GPIO 进入有效电平时提交。

图像触发由 CameraSync 完成；图像与 IMU 的配对由 CameraFrameSync 完成。

## 功能边界

- 每个 Webots step 发布一组原始 IMU 样本。
- 注册一个 `LibXR::GPIO` 作为相机触发线，CameraSync 写该 GPIO 后才提交图像。
- `fps` 只控制 Webots Camera 的底层采样周期，实际图像提交周期由 CameraSync 的分频决定。
- 图像写入 `CameraBase::ImageFrame`，像素格式固定为紧密排列 BGR8。
- 传感器时间戳来自 libxr Webots timebase，不直接读取 `robot->getTime()`。
- 同步命令、分频拉长、seq 回执和 Host/MCU SharedTopic 边界不在本模块处理。
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
| `raw_topic_domain_name` | `mcu` | 原始 IMU topic domain；detector-only 本进程调试可改为默认域。 |
| `trigger_gpio_name` | `CAMERA` | 注册给 CameraSync 查找的相机触发 GPIO 名称。 |
| `trigger_active_level` | `true` | 触发有效电平，进入该电平的边沿提交图像。 |

## Topic

| Topic | Payload | 时间戳 | 说明 |
| --- | --- | --- | --- |
| `<device_name>_gyro` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | 角速度，单位 rad/s。 |
| `<device_name>_accl` | `Eigen::Matrix<float, 3, 1>` | Topic timestamp | 线加速度，单位 m/s^2。 |
| `<device_name>_quat` | `LibXR::Quaternion<float>` | Topic timestamp | 姿态四元数，顺序 wxyz。 |
| `image_topic_name` | `CameraBase::ImageFrame` | `ImageFrame::timestamp_us` | 原始 BGR8 图像。 |
| `host/gimbal_quat` | `LibXR::Quaternion<float>` | Topic timestamp | C 板同名姿态 topic，数据与 `<device_name>_quat` 同源。 |

默认配置下原始 IMU topic 为 `camera_gyro`、`camera_accl`、`camera_quat`。

## 时间与同步探针

libxr Webots timebase 在每次仿真 step 后推进。WebotsCamera 使用 `LibXR::Timebase::GetMicroseconds()` 作为传感器时间戳。

模块不调用 `robot->getTime()`。

正常图像触发间隔为 CameraSync 的 `trigger_div` 个 IMU 样本。收到同步命令后，CameraSync 会把一段反向电平周期临时拉长，然后在下一个有效电平边沿触发同步点；WebotsCamera 只响应 GPIO 边沿，不读取同步命令。

```text
IMU topic -> CameraSync -> GPIO edge -> WebotsCamera CommitImage()
```

CameraFrameSync 根据 CameraSync 回执 topic 的消息时间戳锁定 IMU 时间轴，再为图像选择对应 IMU。WebotsCamera 不比较图像时间戳和 IMU 时间戳，也不发布同步结果。

## 坐标系

Webots world 必须把 Camera、Gyro、Accelerometer、InertialUnit 安装成同一套坐标系：

- 右手系。
- `x` 向右。
- `y` 向前。
- `z` 向上。

`camera_quat`、`camera_gyro`、`camera_accl`、`host/gimbal_quat` 均使用这套坐标系。角速度正方向遵循各轴右手定则。

模块不做运行时零位标定，也不补偿 world 中的安装误差；坐标错误应在 world 中修正。

## Webots 设备约定

当 `pose_def_name = camera` 时，world 中需要提供：

- `camera_gyro`
- `camera_accelerometer`
- `camera_inertial_unit`

三类 IMU 设备应与 Camera 节点共用同一坐标定义。`camera_quat` 和 `host/gimbal_quat` 均来自 `InertialUnit::getQuaternion()`。

Webots 返回的 xyzw 会在模块内转换为 wxyz。

## 依赖

- `qdu-future/CameraBase`
