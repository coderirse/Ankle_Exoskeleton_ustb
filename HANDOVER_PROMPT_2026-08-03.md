# 交接 Prompt：Ankle Exoskeleton 脚踝外骨骼控制系统

把这个 prompt 直接给接手的 AI，让它先读仓库里的 README、工作日志和关键源码，再继续开发。

---

## 一、项目基本信息

- **项目名**：Ankle Exoskeleton 脚踝外骨骼机器人控制系统
- **仓库**：https://github.com/coderirse/Ankle_Exoskeleton_ustb.git
- **本地路径**：`~/git_/Ankle_Exoskeleton_ustb`
- **分支**：`main`
- **最新 commit**：`7e6d599`（2026-08-03晚: 助力提速 + 0x41预张紧 + STM32心跳固件）
- **开发语言**：C++（控制闭环）+ Python（仅离线调试工具）+ C（STM32 固件）
- **ROS 版本**：ROS2 Humble
- **操作系统**：Ubuntu 22.04

**铁律（README 里有写，必须遵守）**：
- 控制闭环（200Hz）**必须用 C++**，绝对禁止 Python
- Python 只准写离线调试脚本，不准接入实时链路
- CAN 通信必须走原生 `libcontrolcan.so`，不准用 Python 桥

---

## 二、硬件配置

| 设备 | 型号/规格 | 连接方式 | 固定端口 |
|---|---|---|---|
| 电机+驱动器 | 卓誉 JRM-C8078-024BN-BG-ACF-S00 | CANopen via CANalyst-II | `/dev/ankle_can` |
| 关节模组 | 减速比 24，输出端峰值 100 rpm = **600°/s**，峰值扭矩 95 N·m | 鲍登线牵引 | — |
| 力传感器 | D056 + DYMH113 | Modbus RTU 19200 | `/dev/ankle_force` |
| 编码器 | 单圈绝对值编码器 | Modbus RTU 9600 | `/dev/ankle_encoder`（当前失效，临时用 `/dev/ttyUSB2`） |
| 足底开关 | STM32F103 双开关采集板 | UART 9600 | `/dev/ankle_switch` |
| IMU | fdilink_ahrs | USB ACM（当前未接入） | `/dev/ttyACM0` |
| CAN 分析仪 | CANalyst-II (04d8:0053) | USB | `/dev/ankle_can` |

**重要**：
- `scripts/99-ankle.rules` 基于 USB 拓扑 `ID_PATH_TAG`，换拓展坞/接口后符号链接会失效，需重新采集
- 当前 `/dev/ankle_encoder` 已失效，launch 文件里临时写死为 `/dev/ttyUSB2`
- CANalyst-II 若打开失败，先执行 `sudo usbreset 04d8:0053`

---

## 三、软件架构

```
STM32 足底双开关 ──UART──▶ serial_sendCommand_node ──/command_topic──┐
力传感器 ──Modbus RTU──▶ serial_ForceSensor_node ──/Force────────────┤
编码器 ──Modbus RTU──▶ serial_encoder_node ──/angle─────────────────┤→ can_ankleControl_node ──CANalyst-II──▶ 电机驱动器
IMU（当前未接）──────▶ fdilink_ahrs ──/imu, /system_speed───────────┘
```

**关键节点**：
- `serial_sendCommand_node`：读足底开关 UART，发布 `/command_topic`（0x41~0x44）和 `/switch_state`（心跳版当前状态）
- `serial_ForceSensor_node`：读力传感器，发布 `/Force`（Float32，单位 kg，20Hz）
- `serial_encoder_node`：读编码器，发布 `/angle`（Float64，200Hz）
- `can_ankleControl_node`：核心控制，200Hz 状态机 + 力闭环 + 初始化对准

**启动命令**：
```bash
cd ~/git_/Ankle_Exoskeleton_ustb
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch can_ankle start_node.launch.py
```

---

## 四、当前已实现的功能

1. **初始化对准流程 `runInitialAlignment()`**：
   - 力控闭环维持力传感器在 **0.5~2.0 kg**（线松收线、线紧放线）
   - 同时检查：开关闭合（`0x42`）+ 力正常 + 编码器在目标范围 + IMU 水平
   - 四项同时满足并持续 **2 秒** → 打印"初始化完成"
   - 总超时 30 秒

2. **35 kg 急停硬限值**：
   - 力传感器读数 ≥ 35 kg → 立即停电机 + 回 STAND_MODE + `emergency_stop` 锁存
   - 锁存后必须重启节点才能恢复

3. **助力提速改造（未实测）**：
   - `max_speed` 从 180°/s 提到 **600°/s**（电机输出端峰值）
   - **前馈 + PID 复合控制**：`velocity = FF_GAIN×目标扭矩 + PID修正`
   - **0x41 预张紧**：脚跟着地就开始以 60°/s 收线至 2 kg，蹬地时线已绷紧

4. **STM32 心跳固件**：
   - 新固件 `src/stm_switch/足底双开关/User/main_heartbeat.c`
   - 状态变化立即发送 + 每 ~50ms 心跳重发当前状态
   - 串口节点去重，新增 `/switch_state` 话题

---

## 五、当前状态与已知问题

### 当前状态
- 代码已提交并推送到 `origin/main`（commit `7e6d599`）
- **STM32 心跳固件已烧录，但芯片尚未重启运行新固件**（ST-LINK NRST 未连接，烧录后不会自动复位）
- 助力提速改造（前馈 + 600°/s + 预张紧）**尚未实测**

### 已知问题
1. **必须拔插 STM32 USB 线** 新固件才会运行
2. `/dev/ankle_encoder` udev 符号链接失效，编码器临时用 `/dev/ttyUSB2`
3. 编码器偶发跳变（如 -31.73°），`serial_encoder_node.cpp` 无 CRC 校验和超时重试
4. IMU 未接入，`/imu` 无数据，坡度补偿和 IMU 水平检测被跳过
5. 力传感器静止读数偏大（~6.9 kg），需确认是安装预载还是零点漂移
6. `level_encoder_target=-22.14°` 是假肢静止站立时的实测值，若换装置需重新标定

---

## 六、接下来的任务清单（按优先级）

### P0：立即验证
1. **拔插 STM32 USB 线**，让心跳固件启动
2. 验证心跳：
   ```bash
   stty -F /dev/ankle_switch 9600 raw -echo
   timeout 5 od -A x -t x1z /dev/ankle_switch
   ```
   应看到每 ~50ms 一个字节，当前假肢站立状态应为 `42`
3. 验证 LED：踩下开关灯亮，松开灯灭

### P1：完整初始化流程验证
1. 假肢静止站立（双脚踩地）
2. 启动全链路：
   ```bash
   ros2 launch can_ankle start_node.launch.py
   ```
3. 期望看到：
   - `对准中: 开关[OK] 力[OK x.xx] 编码器[OK x.xx] IMU[无数据]`
   - 2 秒后打印 **"初始化完成"**
4. 如果仍超时，检查：
   - 开关是否稳定输出 `0x42`
   - 力是否在 0.5~2.0 kg
   - 编码器是否在 -22.14±10°

### P2：助力性能实测与调参（核心）
1. 让假肢走路，观察：
   - 电机转速是否够：`/torque_info` 的 `velocity_value` 是否接近 600 上限
   - 助力时机是否滞后：蹬地瞬间是否立即有拉力
   - 力度是否合适
2. 调参指南：
   - **太冲/太猛**：降 `ff_gain`（当前 15，可试 8~10）
   - **太弱/跟不上**：提 `ff_gain`（可试 20~25），或检查 `T_max` 峰值扭矩上限
   - **预张紧过慢/过快**：调 `pretension_speed`（当前 60）

### P3：稳定性加固
1. **更新 udev 规则**：
   - 重新采集各设备的 `ID_PATH_TAG`
   - 更新 `scripts/99-ankle.rules`
   - 恢复 `/dev/ankle_encoder` 符号链接
2. **编码器读数加固**：
   - 给 `serial_encoder_node.cpp` 加 CRC 校验和超时重试
   - 滤波抑制跳变（如滑动平均或中值滤波）
3. **力传感器零点标定**：
   - 上电后空载状态记录零点
   - 或在初始化前自动清零

### P4：IMU 接入（可选）
1. 接入 fdilink_ahrs
2. 启用 `initializeSlopeFile()` 坡度补偿
3. 让 IMU 水平检测真正参与初始化判定

---

## 七、关键文件路径

| 文件 | 作用 |
|---|---|
| `src/can_ankle/src/can_ankleControl_node.cpp` | 核心控制节点：状态机、初始化对准、PID+前馈、急停 |
| `src/can_ankle/src/serial_sendCommand_node.cpp` | 足底开关节点，新增 `/switch_state` 心跳话题 |
| `src/can_ankle/src/serial_encoder_node.cpp` | 编码器节点（需加固 CRC/重试） |
| `src/can_ankle/src/serial_ForceSensor_node.cpp` | 力传感器节点 |
| `src/can_ankle/launch/start_node.launch.py` | 启动文件和参数配置 |
| `src/can_ankle/include/can_ankle/can_ankle_node.h` | CAN 底层封装 |
| `src/stm_switch/足底双开关/User/main_heartbeat.c` | STM32 心跳固件源码 |
| `scripts/99-ankle.rules` | udev 持久化规则（待更新） |
| `docs/WORK_LOG_2026-08-03.md` | 最新工作日志 |
| `docs/HANDOVER_2026-07-29.md` | 7月29日交接文档 |
| `docs/STATUS_2026-07-29.md` | 7月29日踩坑记录 |

---

## 八、调试命令速查

```bash
# 检查设备
lsusb | grep 04d8:0053          # CANalyst-II
ls -la /dev/ankle_*             # 符号链接

# CAN 复位
sudo usbreset 04d8:0053

# 启动全链路
ros2 launch can_ankle start_node.launch.py

# 监控关键话题
ros2 topic echo /command_topic
ros2 topic echo /switch_state
ros2 topic echo /Force
ros2 topic echo /angle
ros2 topic echo /torque_info

# 监听足底开关原始数据
stty -F /dev/ankle_switch 9600 raw -echo
timeout 5 od -A x -t x1z /dev/ankle_switch

# 编译
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select can_ankle

# STM32 固件编译烧录（在 User 目录）
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2 -nostdlib \
  -T ../link.ld main_heartbeat.c -o main_heartbeat.elf
arm-none-eabi-objcopy -O binary main_heartbeat.elf main_heartbeat.bin
st-flash --reset write main_heartbeat.bin 0x8000000

# 停止所有节点
bash scripts/stop_all.sh --auto
```

---

## 九、安全红线

1. **控制闭环不准用 Python**
2. **`motor_remote.py` 会强杀 CAN 占用进程，不准与控制节点同时运行**
3. **35 kg 急停锁存后必须重启节点才能恢复**
4. 真人上脚测试前，必须先通过假肢/假人脚验证，且必须有保护人员
5. 上脚前检查机械结构、鲍登线、绑带、急停方案

---

## 十、给接手 AI 的指令

请按以下顺序接手：

1. 先读 `README.md`、`docs/WORK_LOG_2026-08-03.md`、`docs/HANDOVER_2026-07-29.md`
2. 查看当前 git 状态，确认在 `main` 分支、commit `7e6d599`
3. 根据"接下来的任务清单"从 P0 开始执行
4. 每次改代码前先 `colcon build --symlink-install --packages-select can_ankle` 编译验证
5. 每次有重大进展更新 `docs/WORK_LOG_2026-08-03.md` 或新建日期工作日志，并提交到仓库
6. 遇到硬件问题（如 CAN 识别不到、串口失效）先查 udev 规则和 `lsusb`

当前最紧急的事：**拔插 STM32 USB 让心跳固件启动，然后验证初始化流程是否能完成。**
