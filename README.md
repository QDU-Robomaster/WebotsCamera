# WebotsCamera

`WebotsCamera` 是这条链路里的“仿真下位机相机/IMU 端点”。

它的目标不是偷懒直接喂同步后结果，而是尽量把 Webots 里的行为做得接近真实硬件：

- 每个 world step 都发布一组原始 IMU
  - `camera_gyro`
  - `camera_accl`
  - `camera_quat`
- 原始 IMU payload 只放测量值，采样时刻通过 Topic timestamp 携带
- 图像只写入 `CameraBase::ImageFrame`，`timestamp_us` 使用传感器侧时间
- 接收一次性 `sensor_sync_cmd` 探针，把**下一次图像周期**从 `N` 拉成 `2N`

## Topic 约定

- 原始 IMU
  - `camera_gyro`：`std::array<float, 3>`，角速度，单位 rad/s
  - `camera_accl`：`std::array<float, 3>`，线加速度，单位 m/s^2
  - `camera_quat`：`std::array<float, 4>`，姿态四元数，顺序 wxyz
- 下行同步探针命令
  - `sensor_sync_cmd`
- 旋转姿态
  - `gimbal/rotation`

## 当前语义

- IMU 发布频率 = Webots basicTimeStep
- 图像发布频率 = `fps` 量化后的整数 step 分频
- 二者保持整数倍关系，便于主机侧做同步探针实验
- `sensor_sync_cmd` 不是持续配置，而是一次性命令
- 探针生效后自动恢复基础周期，不保留 phase 状态
- 运行参数里的名字字段使用 `std::string_view` 作为构造入口；构造完成后模块只保留
  基类自有名字和派生出的 topic / Webots 设备名，不要求调用侧继续持有原字符串
- 模块按控制器进程生命周期使用；启动后启用 Webots 设备，析构路径只停止采集线程，不承诺在同一进程内反复卸载/重载设备。

## 同步探针流程

- 正常状态下，图像按基础周期 `N` step 发布
- 主机侧发出一次 `sensor_sync_cmd`
- `WebotsCamera` 只把“下一张图”的发布时间再向后推一个基础周期
  - 间隔变化表现为 `N -> 2N -> N`
- 主机侧只使用图像传感器时间差观察这个节拍变化
- 最终 IMU 选帧仍由主机侧在 IMU 自己的时间域里完成，`WebotsCamera` 不负责跨域对齐

## 坐标与姿态

- 对外发布的 IMU/姿态坐标系固定为右手系
  - `x` 向右
  - `y` 向前
  - `z` 向上
- `rotation_wxyz`、`camera_gyro`、`camera_accl`、`gimbal/rotation` 全部使用这一套坐标语义
- Webots world 需要在相机同一坐标系下提供三类设备：
  - `camera_gyro`
  - `camera_accelerometer`
  - `camera_inertial_unit`
- `camera_quat` 与 `gimbal/rotation` 来自 `InertialUnit`，不再由 supervisor 直接读取 node 姿态拼出来
- `angular_velocity_xyz` 的正方向遵循各轴右手定则
- 第一次读到相机姿态时会冻结一份零位标定矩阵
  - 后续发布的四元数、角速度、线加速度都相对同一零位解释
  - 不会在运行过程中切换坐标语义

## 依赖

- `qdu-future/CameraBase`
