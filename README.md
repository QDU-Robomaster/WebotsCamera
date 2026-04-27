# WebotsCamera

`WebotsCamera` 是这条链路里的“仿真下位机相机/IMU 端点”。

它的目标不是偷懒直接喂同步后结果，而是尽量把 Webots 里的行为做得接近真实硬件：

- 每个 world step 都发布一组原始 IMU
  - `camera_gyro`
  - `camera_accl`
  - `camera_quat`
- 图像 payload 走 `CameraBase` 的图像 sink
- 每发一张图像，额外发布一条轻量 `image_event`
- 接收一次性 `sensor_sync_cmd` 探针，把**下一次图像周期**从 `N` 拉成 `2N`
- 探针帧会把 `cmd_id` 回填到 `ImageEvent.sync_cmd_id`

## 当前语义

- IMU 发布频率 = Webots basicTimeStep
- 图像发布频率 = `fps` 量化后的整数 step 分频
- 二者保持整数倍关系，便于主机侧做同步探针实验
- `sensor_sync_cmd` 不是持续配置，而是一次性命令
- 探针生效后自动恢复基础周期，不保留 phase 状态

## 坐标与姿态

- 发布姿态采用当前系统接受的 SP/BMI088 风格中立帧
  - `x` 向右
  - `y` 向前
  - `z` 向上
- `rotation / gyro / accel` 三者都会统一到同一发布坐标语义
- 第一次读到相机姿态时会冻结一份零位标定矩阵，后续保持一致

## 依赖

- `qdu-future/CameraBase`
