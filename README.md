# Ankle Exoskeleton Control System

基于 ROS2 Humble 的脚踝外骨骼机器人控制系统，包含 CAN 总线电机控制与 FDILink AHRS 惯性导航传感器驱动。

## ⚠️ 实时性原则

**控制闭环必须使用 C++ 实现，坚决不能使用 Python。**

原因：
- Python 受 GIL 限制，无法保证 200Hz 实时控制周期的确定性
- Python 的垃圾回收会在随机时刻引入延迟尖峰
- USB-CAN 通信、力传感器读取、编码器解码、PID 计算均在微秒级完成，Python 的解释开销不可接受
- 已删除所有 Python 控制链路代码（`assist_controller.py`、`switch_node.py`、`can_bridge.py`），仅保留调试工具

控制链路全部由 C++ ROS2 节点 + STM32 固件组成，Python 仅限离线调试脚本。

## 项目结构

```
ankle_ws/
├── src/
│   ├── can_ankle/              # CAN 总线脚踝关节电机控制包
│   │   ├── include/            # 头文件 (controlcan.h, can_ankle_node.h)
│   │   ├── src/                # C++ 源代码（控制闭环节点）
│   │   ├── launch/             # 启动文件 (Python)
│   │   ├── lib/                # 第三方 CAN 库 (libcontrolcan.so)
│   │   └── msg/                # 自定义 ROS 消息 (Torque.msg)
│   ├── fdilink_ahrs/           # FDILink AHRS IMU 传感器驱动包 (5A A5协议)
│   │   ├── include/            # 头文件
│   │   ├── src/                # C++ 源代码
│   │   └── launch/             # 启动文件 (Python)
│   └── stm_switch/             # STM32 足底双开关固件 (C, Keil MDK)
│       └── 足底双开关/
│           ├── Hardware/       # 硬件驱动层 (I2C, MPU6050, PWM, PID等)
│           ├── Library/        # STM32 标准外设库
│           ├── System/         # 系统层 (Delay, UART等)
│           └── User/           # 主程序 & 中断
├── scripts/                    # 启动脚本 & 调试工具（非实时，不参与控制闭环）
│   ├── start_all.sh            # 一键启动全链路 C++ 节点
│   ├── stop_all.sh             # 停止所有节点
│   ├── 99-ankle.rules          # USB/串口持久化 udev 规则
│   ├── motor_remote.py         # 电机键盘遥控器（独立调试用，不用于助力）
│   └── switch_monitor.py       # 足底开关独立监控（自动扫端口）
├── data/                       # 实验数据
│   ├── lzfirst/                # 编码器/扭矩实验记录
│   └── *.bag, *.csv            # ROS 录包及数据文件
├── docs/                       # 文档
│   ├── note/                   # 传感器协议 & 操作手册
│   └── ros通信架构.png          # ROS 通信架构图
├── build/                      # 编译输出 (colcon)
├── install/                    # 安装目录 (colcon)
└── log/                        # 编译日志 (colcon)
```

## 控制闭环架构（全 C++）

```
足底开关 STM32 ──UART──> serial_sendCommand_node ──/command_topic──> can_ankle_node
                                      │                                    │
                                      │ C++                                │ C++
                                      │                                    │
力传感器 ──Modbus RTU──> serial_ForceSensor_node ──/Force──┤              │
编码器   ──Modbus RTU──> serial_encoder_node ───/angle───┘              │
                                                                          │
                              can_ankle_node ──controlcan 原生库──> CAN 总线 ──> 电机驱动器
                                      │
                                      │ C++
                                      │
                  can_ankleControl_node (CANopen 配置, 模式切换)
```

### 关键节点说明

| 节点 | 语言 | 功能 | 实时性要求 |
|------|------|------|-----------|
| `serial_sendCommand_node` | C++ | 读取足底开关 UART，发布步态时相 `/command_topic` | 1kHz 轮询 |
| `serial_ForceSensor_node` | C++ | Modbus RTU 力传感器 | 高频 |
| `serial_encoder_node` | C++ | Modbus RTU 编码器 | 高频 |
| `can_ankle_node` | C++ | **核心助力控制**：状态机、扭矩曲线、PID 力闭环 | **200Hz 硬实时** |
| `can_ankleControl_node` | C++ | CANopen 电机配置、模式切换 | 指令级 |
| `can_ankleTorqueDriver_node` | C++ | 力矩模式电机驱动 | 高频 |
| `can_ankleVelocity_node` | C++ | 速度模式电机驱动 | 高频 |
| STM32 足底双开关 | C | 足底压力状态机 (0x41~0x45)，UART 输出 | 固件级实时 |

## 硬件连接与端口映射

为避免每次重启后 `/dev/ttyUSB*` 序号变化，使用 udev 规则固定为符号链接：

| 设备 | 固定端口 | 协议/速率 | 说明 |
|------|---------|----------|------|
| 编码器 | `/dev/ankle_encoder` | Modbus RTU / 9600 | 脚踝关节角度 |
| 足底开关 | `/dev/ankle_switch` | UART / 9600 | STM32 状态机输出 0x41~0x45 |
| 力传感器 | `/dev/ankle_force` | Modbus RTU / 19200 | D056 + DYMH113 |
| CAN 分析仪 | `/dev/ankle_can` | USB-CAN | CANalyst-II (04d8:0053) |

**安装 udev 规则（仅需一次）：**
```bash
sudo cp scripts/99-ankle.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -la /dev/ankle_*          # 确认符号链接生成
```

> 注：三个串口设备均为 CH340，无法通过唯一序列号区分，当前规则基于 USB 拓扑（拓展坞端口）。若更换 USB 口或拓展坞，需重新采集 `ENV{ID_PATH_TAG}` 并更新规则。

## 软件包说明

### can_ankle — 脚踝电机 CAN 总线控制

**启动方式（推荐）：**
```bash
ros2 launch can_ankle start_node.launch.py
```

或一键脚本：
```bash
bash scripts/start_all.sh          # 控制节点前台运行，Ctrl+C 停止
bash scripts/stop_all.sh --auto    # 停止所有残留节点
```

启动的节点：
1. `serial_ForceSensor_node` — 力传感器
2. `serial_encoder_node` — 编码器
3. `serial_sendCommand_node` — 足底开关
4. `can_ankleControl_node` — 主控制节点（含开机预紧）

### fdilink_ahrs — FDILink 惯性导航传感器驱动 (5A A5协议)

| 发布话题 | 消息类型 | 说明 |
|---------|---------|------|
| `/imu` | `sensor_msgs/Imu` | IMU 数据（四元数） |
| `/mag_pose_2d` | `geometry_msgs/Pose2D` | 地磁北二维朝向角 |
| `/euler_angles` | `geometry_msgs/Vector3` | 欧拉角 |
| `/magnetic` | `geometry_msgs/Vector3` | 磁力计 |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS 定位 |
| `/system_speed` | `geometry_msgs/Twist` | 机体系速度 |
| `/NED_odometry` | `nav_msgs/Odometry` | NED 系位移与速度 |

**启动方式：**
```bash
ros2 launch fdilink_ahrs ahrs_data.launch.py
ros2 launch fdilink_ahrs tf.launch.py
```

## 环境要求

| 依赖 | 说明 |
|------|------|
| Ubuntu 22.04 | 操作系统 |
| ROS2 Humble | 机器人操作系统 |
| Eigen3 | 线性代数库 |
| wjwwood/serial | 串口通信库 |
| libcontrolcan | VCI USBCAN 原生驱动 |
| usbreset | USB 设备复位工具（通常含于 `usbutils`） |
| STM32F103 | 足底开关 MCU (Keil MDK) |

**安装系统依赖：**
```bash
sudo apt install libeigen3-dev usbutils
# wjwwood/serial 需从源码安装: https://github.com/wjwwood/serial
```

## 编译

```bash
cd ankle_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## ROS 通信架构

项目包含传感器采集、CAN 总线电机控制、数据记录三个环节的 ROS 节点通信拓扑。

![ROS通信架构](docs/ros通信架构.png)

## 上脚测试注意事项

1. **台架与真机的区别**：当前台架编码器未与电机机械联动，因此 `0x44`（摆动相）不会自动松线；上脚后编码器随踝关节运动，摆动相释线逻辑才会生效。
2. **预紧**：开机后控制节点自动慢速收线，直到力传感器读数 ≥ `preload_force`（默认 0.2）。
3. **力保护**：力传感器读数 ≥ `force_limit`（默认 5.0）时电机立即停止。
4. **安全**：首次上脚建议降低 `force_limit` 和 `max_speed`，观察步态切换是否顺畅。

## 实验数据

`data/` 目录下的 `.bag` 文件为 ROS 实验数据录包，包含不同控制参数下的脚踝外骨骼运行数据。

## 开发原则

1. **控制闭环一律 C++** — 任何参与 200Hz 控制循环的代码不得使用 Python
2. **Python 仅限离线工具** — 调试脚本、数据后处理、参数标定允许 Python，但不得接入实时链路
3. **CAN 通信走原生库** — C++ 节点通过 `libcontrolcan.so` 直接操作 CAN 总线，不经过任何中间桥接
4. **STM32 负责底层采集** — 足底开关、AHRS 原始数据采集在 STM32 端完成，通过 UART 发送
