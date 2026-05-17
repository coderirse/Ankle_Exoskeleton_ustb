# Ankle Exoskeleton Control System

基于 ROS Melodic 的脚踝外骨骼机器人控制系统，包含 CAN 总线电机控制与 FDILink AHRS 惯性导航传感器驱动。

## 项目结构

```
ankle_ws/
├── src/
│   ├── can_ankle/          # CAN 总线脚踝关节电机控制包
│   │   ├── include/        # 头文件 (controlcan.h, can_ankle_node.h)
│   │   ├── src/            # 源代码
│   │   ├── launch/         # 启动文件
│   │   ├── lib/            # 第三方 CAN 库 (libcontrolcan.so)
│   │   └── msg/            # 自定义 ROS 消息 (Torque.msg)
│   └── fdilink_ahrs/       # FDILink AHRS IMU 传感器驱动包
│       ├── include/        # 头文件
│       ├── src/            # 源代码
│       └── launch/         # 启动文件
├── build/                  # 编译输出
├── devel/                  # 开发环境配置
└── *.bag                   # 实验数据录包
```

## 软件包说明

### can_ankle — 脚踝电机 CAN 总线控制

| 节点名称 | 功能描述 |
|---------|---------|
| `can_ankleTorqueDriver_node` | 力矩模式电机驱动 |
| `can_ankleVelocity_node` | 速度模式电机驱动 |
| `can_ankleControl_node` | 电机综合控制（含 CANopen 配置、模式切换、数据记录） |
| `can_test_node` | CAN 通信测试节点 |
| `serial_ForceSensor_node` | 串口力传感器数据采集 |
| `serial_encoder_node` | 串口编码器数据采集 |
| `serial_sendCommand_node` | 串口指令发送 |
| `storeTopicMSG_node` | ROS 话题数据记录 |
| `storeVelMSG_node` | 速度数据记录 |

**启动方式：**

```bash
roslaunch can_ankle start_node.launch
```

### fdilink_ahrs — FDILink 惯性导航传感器驱动

基于 FDILink Deta-10 AHRS/INS 传感器的 ROS 驱动，通过串口读取传感器数据并发布为 ROS 话题。

| 发布话题 | 消息类型 | 说明 |
|---------|---------|------|
| `/imu` | `sensor_msgs/Imu` | IMU 数据（四元数、角速度、线加速度） |
| `/mag_pose_2d` | `geometry_msgs/Pose2D` | 地磁北二维朝向角 |
| `/euler_angles` | `geometry_msgs/Vector3` | 欧拉角（横滚/俯仰/偏航） |
| `/magnetic` | `geometry_msgs/Vector3` | 磁力计磁场强度 |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS 定位数据 |
| `/system_speed` | `geometry_msgs/Twist` | 机体系速度 |
| `/NED_odometry` | `nav_msgs/Odometry` | NED 系位移与速度 |

**启动方式：**

```bash
# 启动 AHRS 传感器驱动
roslaunch fdilink_ahrs ahrs_data.launch

# 启动 IMU 坐标变换（TF）
roslaunch fdilink_ahrs tf.launch
```

## 环境要求

| 依赖 | 说明 |
|------|------|
| Ubuntu 18.04 | 操作系统 |
| ROS Melodic | 机器人操作系统 |
| Eigen3 | 线性代数库 |
| `ros-melodic-serial` | ROS 串口通信包 |

**安装系统依赖：**

```bash
sudo apt install ros-melodic-serial libeigen3-dev
```

## 编译

```bash
cd ankle_ws
catkin_make
source devel/setup.bash
```

## ROS 通信架构

项目包含传感器采集、CAN 总线电机控制、数据记录三个环节的 ROS 节点通信拓扑。

![ROS通信架构](ros通信架构.png)

## 实验数据

根目录下的 `.bag` 文件为 ROS 实验数据录包，包含不同控制参数下的脚踝外骨骼运行数据，可使用 `rosbag play` 或 `rqt_bag` 进行回放分析。
