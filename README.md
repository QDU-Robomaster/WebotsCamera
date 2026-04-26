# WebotsCamera

`WebotsCamera` 是当前 Webots / Linux 自瞄链路里的仿真相机模块。

它负责从 Webots `Camera` 读取原始图像，按目标频率发布图像与 IMU，
并把相机姿态转换成当前链路约定的 `gimbal/rotation` 与 `CameraBase`
前半段数据边界。

## 运行时职责

- 从 Webots `Camera` 获取原始 `BGRA` 图像
- 按目标 `fps` 把发布节奏量化到世界步长
- 从 `Supervisor` 查询相机姿态，并通过连续姿态差分估计角速度和线加速度
- 先发布 `ImuStamped`，再把图像写入 `CameraBase<Info>::ImageFrame`
- 通过 `CommitImage()` 把图像交给后续的共享图像桥接模块
- 额外发布 `gimbal/rotation` 话题，供下游沿用现有云台接口

## 当前边界

- 输入依赖：
  - Webots `Robot`
  - Webots `Camera`
  - Webots `Supervisor`
  - `CameraBase<Info>`
- 输出边界：
  - 图像：`CameraBase<Info>::ImageFrame`
  - IMU：`CameraBase<Info>::ImuStamped`
  - 云台旋转：`gimbal/rotation`
- 该模块只负责仿真采样、节流、姿态/运动估计和前半段发布
- 图像共享内存发布与图像/IMU 同步由后续 `CameraFrameSync<Info>` 负责

## 构造参数

- `runtime.device_name`
  - Webots 相机设备名
- `runtime.fps`
  - 目标发布频率
- `runtime.exposure`
  - Webots 相机曝光值
- `runtime.gain`
  - 仅记录请求值；Webots `Camera` 不原生支持 gain
- `runtime.pose_def_name`
  - 用于 `Supervisor` 查询位姿的 DEF 名
- `runtime.image_topic_name`
  - 原始图像输出名
- `runtime.imu_topic_name`
  - IMU 输出名

## 模板参数

- `Info`
  - 编译期 `CameraTypes::CameraInfo`
  - 当前要求：
    - `encoding == CameraTypes::Encoding::BGR8`
    - `step == width * 3`

## 说明

- 当前实现假定输出图像是紧凑的 `BGR8`
- Webots 原始图像格式是 `BGRA`，模块内部会转换成 `BGR`
- 发布顺序固定为：
  - 仿真节流
  - pose
  - motion
  - image
  - sink commit
- `pose` 与图像在同一线程采样，因此角速度和线加速度通过连续姿态差分估计
- 运行时可以通过环境变量 `XR_WEBOTS_CAMERA_EXPOSURE` 覆盖启动曝光

## 依赖

- `qdu-future/CameraBase`
- OpenCV：`core`, `imgproc`, `highgui`
- Webots C++ API
