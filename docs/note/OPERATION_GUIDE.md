# 脚踝外骨骼控制系统 — 操作手册

## 目录

1. [系统概述](#1-系统概述)
2. [硬件连接](#2-硬件连接)
3. [软件环境](#3-软件环境)
4. [编译项目](#4-编译项目)
5. [启动流程](#5-启动流程)
6. [节点与话题速查](#6-节点与话题速查)
7. [运行模式说明](#7-运行模式说明)
8. [常见问题](#8-常见问题)

---

## 1. 系统概述

本项目是基于 ROS Melodic 的脚踝外骨骼机器人控制系统，实现以下核心功能：

- **传感器采集**：IMU 姿态感知、力传感器力矩反馈、编码器关节角度
- **CAN 总线电机控制**：速度/位置/扭矩三种模式，含 CANopen 协议配置
- **自适应步态控制**：根据步速自适应调整支撑相扭矩曲线（上升/下降阶段）
- **安全保护**：编码器限位、力传感器过载、指令序列校验，异常时自动归零

### 硬件清单

| 设备 | 型号 | 接口 |
|------|------|------|
| IMU 惯性导航传感器 | FDILink HI13S4-USB-010 | USB Type-C → PC |
| 力传感器 | 大洋 DYMH113 | 自带线 → D056 仪表 |
| 力传感器显示仪表 | 大洋 D056 | RS485 → CH340 USB模块 → PC |
| 编码器 | (角度传感器) | RS485/串口 → PC |
| CAN 总线电机 | (伺服电机+减速器 100:1) | USB-CAN 适配器 → PC |
| 脚底开关 | (压力开关) | 发布步态指令话题 |

---

## 2. 硬件连接

### 连接拓扑

```
┌─────────────────────────────────────────────────────────────────┐
│                         电脑 (Ubuntu 18.04)                      │
│                                                                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │USB-CAN   │  │CH340     │  │CH340/编码 │  │IMU       │        │
│  │适配器    │  │USB-485   │  │器串口    │  │USB-C    │        │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘        │
│       │CAN总线      │RS485       │串口          │USB            │
│       ▼             ▼            ▼              ▼               │
│  ┌────────┐   ┌─────────┐  ┌────────┐   ┌──────────┐           │
│  │伺服电机│   │D056仪表 │  │编码器  │   │HI13S4    │           │
│  │(CAN)   │   │(RS485)  │  │(串口)  │   │IMU       │           │
│  └────────┘   └────┬────┘  └────────┘   └──────────┘           │
│                    │                                            │
│               ┌────┴────┐                                       │
│               │DYMH113  │                                       │
│               │力传感器 │                                       │
│               └─────────┘                                       │
└─────────────────────────────────────────────────────────────────┘
```

### 端口映射

| 设备 | Ubuntu 设备路径 | 协议 | 波特率 |
|------|----------------|------|--------|
| IMU (HI13S4) | `/dev/wheeltec_FDI_IMU_GNSS` | 自定义二进制帧 (FC 帧头) | 921600 |
| 力传感器 (D056) | `/dev/ttyUSB1` | ASCII 文本 (`\r` 结尾) | 115200 |
| 编码器 | `/dev/ttyUSB0` | Modbus RTU (站号1) | 9600 |
| CAN 电机 | USB-CAN 适配器 | CANopen | 500kbps |

> **Windows 端对应端口**（用于单独测试传感器）：
>
> | 设备 | Windows COM | 协议 | 波特率 |
> |------|------------|------|--------|
> | IMU | COM12 (CH9102) | 5A A5 二进制帧 | 115200 |
> | 力传感器 | COM11 (CH340) | Modbus RTU (站号1, 寄存器 0x0FA1) | 19200 |

---

## 3. 软件环境

| 依赖 | 版本/说明 |
|------|-----------|
| 操作系统 | Ubuntu 18.04 |
| ROS | Melodic |
| C++ 标准 | C++11 |
| Eigen3 | 线性代数库 |
| ros-melodic-serial | ROS 串口通信包 |
| libcontrolcan.so | 第三方 CAN 库 (位于 `can_ankle/lib/`) |

### 安装系统依赖

```bash
sudo apt install ros-melodic-serial libeigen3-dev
```

---

## 4. 编译项目

```bash
cd ~/ankle_ws
catkin_make
source devel/setup.bash
```

编译产物：

| 包 | 生成的可执行文件 |
|----|-----------------|
| fdilink_ahrs | `ahrs_driver`, `imu_tf`, `libcrc_table.so` |
| can_ankle | `can_ankleControl_node`, `serial_ForceSensor_node`, `serial_encoder_node`, `can_ankleTorqueDriver_node`, `can_ankleVelocity_node`, `can_test_node`, `serial_sendCommand_node`, `storeTopicMSG_node`, `storeVelMSG_node` |

---

## 5. 启动流程

### 完整启动（推荐顺序）

每次新开终端需先 source 环境：

```bash
source ~/ankle_ws/devel/setup.bash
```

#### 步骤 1：启动 IMU 传感器驱动

```bash
# 终端 1：启动 AHRS 驱动（发布 /imu、/euler_angles 等话题）
roslaunch fdilink_ahrs ahrs_data.launch

# 终端 2：启动 TF 坐标变换广播
roslaunch fdilink_ahrs tf.launch
```

#### 步骤 2：启动脚踝控制

```bash
# 终端 3：启动主控制 + 力传感器 + 编码器
# 会在 3 个独立 xterm 窗口中分别运行
roslaunch can_ankle start_node.launch
```

`start_node.launch` 实际启动了 3 个节点：

```
xterm 1: can_ankleControl_node      # 主控制（状态机、电机驱动）
xterm 2: serial_ForceSensor_node    # 力传感器读取
xterm 3: serial_encoder_node        # 编码器角度读取
```

#### 步骤 3（按需）：运行辅助节点

```bash
# 扭矩模式单独测试
rosrun can_ankle can_ankleTorqueDriver_node

# 速度模式单独测试
rosrun can_ankle can_ankleVelocity_node

# CAN 通信测试
rosrun can_ankle can_test_node

# 数据记录
rosrun can_ankle storeTopicMSG_node
rosrun can_ankle storeVelMSG_node
```

### 启动后交互

`can_ankleControl_node` 启动后会提示：

```
请输入模式1/2:
```
- **模式 1**：自适应步速扭矩曲线（推荐，根据步速动态调整）
- **模式 2**：固定参数扭矩曲线（上升 0.18s + 下降 0.21s）

```
请输入用户体重(kg,范围40-90):
```
- 用于计算基准峰值扭矩：`基准扭矩 = 体重 × 0.3`

---

## 6. 节点与话题速查

### 节点一览

| 节点名 | 包 | 功能 | 启动方式 |
|--------|-----|------|----------|
| ahrs_driver | fdilink_ahrs | IMU 驱动 | `roslaunch fdilink_ahrs ahrs_data.launch` |
| imu_tf | fdilink_ahrs | TF 广播 | `roslaunch fdilink_ahrs tf.launch` |
| can_ankleControl_node | can_ankle | 主控制（状态机+电机） | `roslaunch can_ankle start_node.launch` |
| serial_ForceSensor_node | can_ankle | 力传感器 | 同上 |
| serial_encoder_node | can_ankle | 编码器 | 同上 |

### 话题通信表

| 话题 | 方向 | 类型 | 发布者 → 订阅者 |
|------|------|------|----------------|
| `/imu` | 发布 | sensor_msgs/Imu | ahrs_driver → can_ankleControl_node, imu_tf |
| `/euler_angles` | 发布 | geometry_msgs/Vector3 | ahrs_driver → 外部 |
| `/system_speed` | 发布 | geometry_msgs/Twist | ahrs_driver → can_ankleControl_node |
| `/Force` | 发布 | std_msgs/Float32 | serial_ForceSensor_node → can_ankleControl_node |
| `/angle` | 发布 | std_msgs/Float64 | serial_encoder_node → can_ankleControl_node |
| `/command_topic` | 订阅 | std_msgs/UInt8 | 脚底开关 → can_ankleControl_node |
| `/one_support_time` | 订阅 | std_msgs/Float64 | 外部 → can_ankleControl_node |
| `/two_support_time` | 订阅 | std_msgs/Float64 | 外部 → can_ankleControl_node |
| `/three_support_time` | 订阅 | std_msgs/Float64 | 外部 → can_ankleControl_node |
| `/swing_time` | 订阅 | std_msgs/Float64 | 外部 → can_ankleControl_node |
| `/torque_info` | 发布 | can_ankle/Torque | can_ankleControl_node → 外部 |

### 脚底开关指令协议

指令通过 `/command_topic` (UInt8) 发送，必须严格遵守顺序：

| 指令 | 含义 | 前置指令 |
|------|------|----------|
| `0x41` | 进入支撑相（扭矩模式） | 0x44 或首次 |
| `0x42` | 预张紧 + 存储坡度 | 0x41 |
| `0x43` | 开始扭矩驱动 | 0x42 |
| `0x44` | 进入摆动相（速度模式归位） | 0x43 |

顺序错误时自动触发安全归零。

---

## 7. 运行模式说明

`can_ankleControl_node` 内含状态机，共 5 种模式：

```
                    ┌─────────────┐
                    │  STAND_MODE │  ← 站立/空闲（电机停转）
                    └──────┬──────┘
                           │ 0x41
                           ▼
                    ┌─────────────┐
                    │ TORQUE_MODE │  ← 记录支撑相起始位置
                    └──────┬──────┘
                           │ 0x42
                           ▼
                    ┌─────────────────┐
                    │ PRE_TORQUE_MODE │  ← 存储坡度、计算峰值扭矩
                    └────────┬────────┘
                             │ 0x43
                             ▼
                    ┌──────────────────┐
                    │ TORQUE_DRIVE_MODE│  ← 执行扭矩曲线驱动
                    │  (支撑相阶段3)   │     上升→峰值→下降
                    └────────┬─────────┘
                             │ 超时或 0x44
                             ▼
                    ┌──────────────┐
                    │VELOCITY_MODE │  ← 摆动相归位
                    └──────┬───────┘
                            │ 到达目标位置
                            ▼
                    ┌─────────────┐
                    │  STAND_MODE │  ← 等待下一次 0x44
                    └─────────────┘
```

### 扭矩曲线（模式 1 — 自适应）

```
扭矩
  ↑
  │        ╱╲
  │       ╱  ╲
  │      ╱    ╲
  │     ╱      ╲
  │    ╱        ╲
  │   ╱          ╲
  └──┴────────────┴────→ 时间
     ←上升段→←下降段→
     rise_time fall_time
     └── DRIVE_DURATION ──┘
```

- 目标峰值扭矩 = 基准扭矩(体重×0.3) + 速度浮动 + 坡度辅助 + 步幅补偿
- PID 控制器将目标扭矩转换为电机速度指令
- 自动根据步速调整 PID 参数 (Kp=20~60, Ki=0.003~0.007, Kd=0.2~0.6)

### 安全机制

| 触发条件 | 动作 |
|----------|------|
| 编码器超出 ±45° | 自动归零，关闭节点 |
| 力传感器 > 300N | 触发归零 |
| 指令序列错误 | 进入 STAND_MODE，触发归零 |
| Ctrl+C | 归零 → 停止电机 → 关闭 CAN 设备 |

---

## 8. 常见问题

### 串口无法打开

```bash
# 检查串口设备是否存在
ls -la /dev/ttyUSB*
ls -la /dev/wheeltec_*

# 添加 udev 规则（如需要）
sudo usermod -a -G dialout $USER
```

### CAN 设备无法打开

```bash
# 检查 CAN 设备
lsusb | grep -i can

# 确认 libcontrolcan.so 在 can_ankle/lib/ 下
ls ~/ankle_ws/src/can_ankle/lib/
```

### ROS 编译错误

```bash
# 确保依赖已安装
sudo apt install ros-melodic-serial ros-melodic-tf ros-melodic-nav-msgs libeigen3-dev

# 清理后重新编译
cd ~/ankle_ws
catkin_make clean
catkin_make
```

### 传感器无数据

| 现象 | 检查项 |
|------|--------|
| IMU 无数据 | 检查 launch 文件中 `port` 参数是否正确，波特率是否 921600 |
| 力传感器无数据 | 检查 `/dev/ttyUSB1` 路径，D056 仪表是否上电，波特率是否 115200 |
| 编码器无数据 | 检查 `/dev/ttyUSB0` 路径，Modbus RTU 接线，波特率是否 9600 |
| 电机不响应 | 检查 CAN 总线终端电阻、电机供电、USB-CAN 适配器指示灯 |

### 单独测试传感器（Windows 端）

参见 [SENSOR_PROTOCOL.md](SENSOR_PROTOCOL.md) 了解传感器通信协议细节。

```bash
# Windows 下测试两个传感器
python vofa_sensor_reader.py --no-tcp --rate 5

# 或在 VOFA+ 中可视化
python vofa_sensor_reader.py --port 1347
```
