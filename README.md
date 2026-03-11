# army_gpd — JetArm GPD 抓取系统

基于 [GPD (Grasp Pose Detection)](https://github.com/atenpas/gpd) 的 6-DOF 机械臂抓取系统，运行于 **ROS Melodic + JetArm (Hiwonder)** 平台。包含点云处理、抓取规划、交互式 UI 三大模块。

---

## 目录结构

```
army_gpd/
├── gpd/                  # GPD 核心库（C++，含预训练模型）
│   ├── cfg/              #   ros_eigen_params.cfg — 主配置文件
│   ├── models/           #   LeNet / Caffe / OpenVINO 模型权重
│   ├── src/              #   C++ 源码
│   └── include/          #   头文件
├── gpd_ros/              # GPD 的 ROS 封装
│   ├── launch/           #   gpd_once.launch, ur5.launch
│   ├── msg/              #   GraspConfig, GraspConfigList, CloudIndexed …
│   ├── srv/              #   detect_grasps.srv
│   └── src/              #   C++ ROS 节点
├── jetarm_ui/            # 控制 UI + 抓取节点 + 点云桥接
│   ├── scripts/          #   Python 脚本（UI、抓取、点云变换）
│   ├── launch/           #   启动文件
│   └── config/           #   YAML 配置
├── user_config/          # 用户级独立配置（不随代码部署覆盖）
├── start_tf_ui.sh        # 一键启动脚本（部署到机械臂桌面）
└── README.md
```

---

## 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                    RGB-D Camera (Astra)                   │
│  /rgbd_cam/depth/points    /rgbd_cam/color/camera_info   │
│  /rgbd_cam/depth/image_raw /color_detection/image_result │
└──────────┬──────────────────────────┬────────────────────┘
           │                          │
     PointCloud2                   CameraInfo / Image
           │                          │
           ▼                          ▼
┌─────────────────────┐   ┌───────────────────────────────┐
│ pointcloud_to_base  │   │  tf_calibration_control_ui    │
│ (TF 变换 + 降采样)  │   │  (PyQt5 交互式控制 UI)        │
│                     │   │  · TF 标定                    │
│ depth → base_link   │   │  · 候选夹取可视化 & 手动选择  │
└────────┬────────────┘   │  · 速度/偏移/参数调节         │
         │                │  · 单次 GPD 扫描触发           │
    PointCloud2           └──────┬────────────────────────┘
    (base_link)                  │ Trigger service
         │                       ▼
         │            ┌─────────────────────┐
         └───────────▶│   gpd_once_server   │  (gpd_ros)
                      │ detect_grasps 服务  │
                      └────────┬────────────┘
                               │ GraspConfigList
                               ▼
                      ┌─────────────────────┐
                      │   gpd_grasp_node    │
                      │ 评分/排序/IK/执行   │
                      │ 斜向推抓 + 避边策略 │
                      └────────┬────────────┘
                               │ MultiRawIdPosDur
                               ▼
                      ┌─────────────────────┐
                      │   JetArm 舵机控制器  │
                      │ /controllers/...    │
                      └─────────────────────┘
```

---

## 需要对接的外部接口

### 1. ROS Topics（需要外部提供）

| Topic | 消息类型 | 提供方 | 说明 |
|-------|---------|--------|------|
| `/rgbd_cam/depth/points` | `sensor_msgs/PointCloud2` | RGB-D 相机驱动 | 原始深度点云 |
| `/rgbd_cam/depth/image_raw` | `sensor_msgs/Image` | RGB-D 相机驱动 | 深度图（UI 显示） |
| `/rgbd_cam/color/camera_info` | `sensor_msgs/CameraInfo` | RGB-D 相机驱动 | 相机内参（投影用） |
| `/color_detection/image_result` | `sensor_msgs/Image` | color_detection 节点 | RGB 画面（UI 显示） |
| `/object/pixel_coords` | `hiwonder_interfaces/ObjectsInfo` | color_detection 节点 | 目标检测结果 |

### 2. ROS Topics（本系统发布）

| Topic | 消息类型 | 发布方 | 说明 |
|-------|---------|--------|------|
| `/rgbd_cam/depth/points_base` | `sensor_msgs/PointCloud2` | `pointcloud_to_base` | base_link 坐标系下的点云 |
| `/detect_grasps/clustered_grasps` | `gpd_ros/GraspConfigList` | `gpd_once_server` | GPD 候选抓取列表 |
| `/controllers/multi_id_pos_dur` | `hiwonder_interfaces/MultiRawIdPosDur` | `gpd_grasp_node` / UI | 舵机运动指令 |
| `/gpd_grasp/debug_markers` | `visualization_msgs/MarkerArray` | `gpd_grasp_node` | RViz 调试标记 |
| `/gpd_once_server/plot_grasps` | `visualization_msgs/MarkerArray` | `gpd_once_server` | GPD 夹爪可视化 |

### 3. ROS Services（需要外部提供）

| Service | 类型 | 提供方 | 说明 |
|---------|------|--------|------|
| `/color_detection/enter` | `std_srvs/Empty` | color_detection 包 | 进入颜色检测模式 |
| `/color_detection/start` | `std_srvs/Empty` | color_detection 包 | 开始检测 |
| `/color_detection/stop` | `std_srvs/Empty` | color_detection 包 | 停止检测 |
| `/color_detection/exit` | `std_srvs/Empty` | color_detection 包 | 退出检测模式 |
| `/rgbd_cam/set_uvc_auto_exposure` | `std_srvs/SetBool` | astra_camera 驱动 | 自动曝光开关 |
| `/rgbd_cam/set_uvc_exposure` | `astra_camera/SetInt32` | astra_camera 驱动 | 手动曝光值 |
| `/rgbd_cam/set_uvc_auto_white_balance` | `std_srvs/SetBool` | astra_camera 驱动 | 自动白平衡开关 |
| `/lab_config_manager/enter` | `std_srvs/Trigger` | lab_config 包 | 加载 LAB 颜色配置 |

### 4. ROS Services（本系统提供）

| Service | 类型 | 提供方 | 说明 |
|---------|------|--------|------|
| `/gpd_once_server/detect_grasps` | `gpd_ros/detect_grasps` | `gpd_once_server` | 单次抓取检测 |
| `/gpd_grasp/trigger` | `std_srvs/Trigger` | `gpd_grasp_node` | 触发一次抓取 |
| `/gpd_grasp/clear_cache` | `std_srvs/Trigger` | `gpd_grasp_node` | 清除缓存的抓取结果 |

### 5. TF 坐标变换（需要外部提供）

| Parent | Child | 提供方 | 说明 |
|--------|-------|--------|------|
| `base_link` | `link1` … `link5` | 机器人 URDF / `robot_state_publisher` | 机械臂关节链 |
| `rgbd_cam_link` | `rgbd_cam_color_optical_frame` | 相机驱动 (astra) | 相机光学坐标系 |
| `rgbd_cam_link` | `rgbd_cam_depth_optical_frame` | 相机驱动 (astra) | 深度光学坐标系 |

> **注意**: `link5 → rgbd_cam_link` 由本系统内的 `static_transform_publisher` 或 UI 的 TF 标定模块发布，参数在 `tf_calibration.yaml` 中配置。

### 6. ROS Actions（需要外部提供）

| Action | 类型 | 提供方 | 说明 |
|--------|------|--------|------|
| `/grasp` | `hiwonder_interfaces/MoveAction` | JetArm 运动控制器 | 机械臂轨迹执行 |

### 7. 舵机接口

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/controllers/multi_id_pos_dur` | `hiwonder_interfaces/MultiRawIdPosDur` | 发送舵机 ID + 位置 + 时长 |

舵机编号：ID 1–5 为关节舵机，ID 10 为夹爪舵机。

### 8. 外部 ROS 包依赖

| 包名 | 用途 | 安装方式 |
|------|------|---------|
| `jetarm_bringup` | 机器人基础启动 (base.launch) | JetArm SDK |
| `jetarm_peripherals` | 相机启动 (camera.launch) | JetArm SDK |
| `jetarm_driver` | 舵机驱动 | JetArm SDK |
| `hiwonder_interfaces` | 自定义消息/动作 | JetArm SDK |
| `color_detection` | 颜色检测节点 | JetArm SDK |
| `lab_config` | LAB 颜色配置管理 | JetArm SDK |
| `astra_camera` | Astra RGB-D 驱动 | `ros-melodic-astra-camera` |
| `jetarm_6dof_moveit_config` | MoveIt 配置 (RViz 可视化) | JetArm SDK |
| `tf2_ros` / `tf2_sensor_msgs` | TF 坐标变换 | `ros-melodic-tf2-*` |

### 9. 系统级依赖

| 库 | 版本要求 | 用途 |
|----|---------|------|
| **PCL** | ≥ 1.9 | 点云处理 |
| **Eigen** | ≥ 3.0 | 线性代数运算 |
| **OpenCV** | ≥ 3.4 | 图像处理 / CNN 推理 |
| **Python 3** + PyQt5 | — | UI 界面 |
| **ROS Melodic** | — | 基础框架 |

---

## 快速部署

### 1. 编译 GPD 核心库

```bash
cd gpd
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 2. 编译 ROS 包

将 `gpd_ros` 和 `jetarm_ui` 放入 catkin 工作空间：

```bash
cp -r gpd_ros jetarm_ui ~/jetarm/src/
cd ~/jetarm
catkin_make
source devel/setup.bash    # 或 setup.zsh
```

### 3. 配置

- **相机标定**: 编辑 `jetarm_ui/config/tf_calibration.yaml` 中的 `transform` 字段
- **抓取偏移**: 编辑 `grasp_offset` 字段，或在 UI 中实时调节
- **GPD 参数**: 编辑 `gpd/cfg/ros_eigen_params.cfg`（workspace、采样数等）
- **模型路径**: 确保 `weights_file` 指向正确的 `models/lenet/15channels/params/` 路径

### 4. 启动

```bash
# 方式一：使用启动脚本（推荐）
chmod +x start_tf_ui.sh
./start_tf_ui.sh

# 方式二：手动 roslaunch
roslaunch jetarm_ui tf_calibration_control_ui.launch
```

---

## 关键配置文件

| 文件 | 说明 |
|------|------|
| `gpd/cfg/ros_eigen_params.cfg` | GPD 核心参数（workspace、采样数、CNN 通道数等） |
| `jetarm_ui/config/tf_calibration.yaml` | TF 标定值 + 抓取偏移 + GPD 调优参数 |
| `jetarm_ui/config/ui_config.yaml` | UI 默认配置（相机 topic、机械臂姿态等） |
| `user_config/ui_config.user.yaml` | 用户独立配置（不随代码更新覆盖） |

---

## GPD 核心参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `num_samples` | 40 | 从点云中采样的候选点数 |
| `num_threads` | 4 | CPU 线程数 |
| `workspace` | `-0.05 0.36 -0.30 0.30 -0.05 0.40` | 点云裁剪范围 (m) |
| `workspace_grasps` | (同上) | 抓取候选过滤范围 |
| `num_selected` | 20 | 最终输出的抓取候选数 |
| `image_num_channels` | 15 | CNN 输入通道数 |
| `weights_file` | `models/lenet/15channels/params/` | 模型权重路径 |

---

## 自定义消息/服务 (gpd_ros)

### 消息

| 消息 | 字段 |
|------|------|
| `GraspConfig` | `position`, `approach`, `binormal`, `axis`, `width`, `score`, `sample` |
| `GraspConfigList` | `header`, `grasps[]` |
| `CloudIndexed` | `cloud_sources`, `indices[]` |
| `CloudSources` | `cloud`, `camera_source[]`, `view_points[]` |

### 服务

| 服务 | 请求 | 响应 |
|------|------|------|
| `detect_grasps` | `CloudIndexed cloud_indexed` | `GraspConfigList grasp_configs` |

---

## License

GPD 核心库遵循 BSD 协议。其余代码仅供学习研究使用。
