# 传感器通信协议文档

## 硬件连接

| 传感器 | 型号 | 连接方式 | 电脑端口 |
|--------|------|----------|----------|
| IMU (惯性导航) | FDILink HI13S4-USB-010 | USB 转 Type-C | COM12 |
| 力传感器 | 大洋 DYMH113 + 大洋 D056 显示控制仪 | RS485 转 USB (CH340) | COM11 |

> 连接拓扑：**力传感器 DYMH113 --(自带信号线)--> D056 显示控制仪表 --(RS485 两线)--> CH340 USB转485 模块 --> 电脑**

---

## 1. 力传感器 (D056 + DYMH113)

### 通信参数

| 参数 | 值 |
|------|-----|
| 物理接口 | RS485 |
| 协议 | **Modbus RTU** |
| 波特率 | **19200** |
| 数据位 | 8 |
| 校验位 | None |
| 停止位 | 1 |
| 从站地址 | **1** |

### 读取当前力值

- **功能码**: `0x03` (Read Holding Registers)
- **寄存器地址**: `0x0FA1` (十进制 4001)
- **数据类型**: **有符号 16 位整数** (signed int16), 大端字节序
- **Modbus 请求帧**: `01 03 0F A0 00 02 [CRC16]`
  - 读取从 0x0FA0 开始的 2 个寄存器 (0x0FA0 和 0x0FA1)
- **Modbus 响应帧**: `01 03 04 [HH] [HL] [LH] [LL] [CRC16]`
  - 其中 [LH] [LL] 两个字节组成有符号 16 位力值

### 力值换算

```
显示值 = 原始值 × scale_factor
```

- 原始值为有符号 16 位整数 (如 -55)
- 显示器显示约 -0.60，对应 raw = -55
- 比例系数 ≈ 0.01 (即 raw/100 ≈ 显示值)

### 配置参数 (低地址段，仅供参考)

D056 的配置参数存储在寄存器 0x0000-0x003C 区域，以 32 位大端 float 格式存储：

| 寄存器地址 | 含义 (推测) | 典型值 |
|-----------|-------------|--------|
| 0x0000 | 量程1 | 20.0 |
| 0x0002 | 量程2 | 50.0 |
| 0x0004 | 量程3 | 100.0 |
| 0x000C | 分辨率 | 0.1 |
| 0x0018 | 阈值 | 0.5 |

### CRC16 计算 (Modbus RTU)

```python
def modbus_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc  # 小端字节序，低字节在前
```

### 示例 Python 读取代码

```python
import serial, struct

def read_force(ser):
    """读取力传感器当前值，返回有符号16位整数"""
    request = bytes([1, 0x03, 0x0F, 0xA0, 0, 2])
    request += modbus_crc(request).to_bytes(2, 'little')
    ser.reset_input_buffer()
    ser.write(request)
    time.sleep(0.06)
    response = ser.read(256)
    if len(response) >= 7:
        # 验证 CRC
        if modbus_crc(response[:-2]).to_bytes(2, 'little') == response[-2:]:
            # 提取寄存器 0x0FA1 的值
            reg1_hi, reg1_lo = response[5], response[6]
            raw = (reg1_hi << 8) | reg1_lo
            if raw >= 0x8000:  # 转为有符号
                raw -= 0x10000
            return raw
    return None

ser = serial.Serial("COM11", 19200, timeout=0.3)
force_raw = read_force(ser)
force_display = force_raw / 100.0  # 近似显示值
```

---

## 2. IMU (FDILink HI13S4-USB-010)

### 通信参数

| 参数 | 值 |
|------|-----|
| 物理接口 | USB (Type-C) |
| 协议 | **自定义二进制帧** |
| 波特率 | **115200** |
| 数据位 | 8 |
| 校验位 | None |
| 停止位 | 1 |

### 帧结构

每个完整帧 **82 字节**：

```
┌──────────┬──────────┬──────────┬─────────────────┬──────────┐
│  Header  │  Length  │ Payload  │    Trailer      │  Next    │
│  2 bytes │  2 bytes │ 76 bytes │    2 bytes      │  Frame   │
│  5A A5   │ LE u16   │  data    │  (constant?)    │  ...     │
└──────────┴──────────┴──────────┴─────────────────┴──────────┘
```

- **Header**: 固定 `0x5A 0xA5` (小端序)
- **Length**: 16 位小端无符号整数，值为 76 (0x004C)，表示 Payload 长度
- **Payload**: 76 字节数据体
- **Trailer**: 2 字节帧尾（可能为校验和）

### Payload 数据布局

76 字节 Payload 以**小端序 float32** 存储传感器数据：

| 偏移 | 字节 | 数据类型 | 内容 | 说明 |
|------|------|----------|------|------|
| 0-1 | 2B | uint16 | 帧计数器 | 递增 |
| 2-3 | 2B | uint16 | 设备ID | 固定 0x0091 |
| 4-13 | 10B | — | 头部/状态 | 未完全解析 |
| 14-17 | 4B | float32 | 未知浮点 | |
| **18-21** | **4B** | **float32** | **四元数 w** | 通常接近 1.0 |
| **22-25** | **4B** | **float32** | **四元数 x** | |
| **26-29** | **4B** | **float32** | **四元数 y** | |
| **30-33** | **4B** | **float32** | **四元数 z** | |
| 34-73 | 40B | float32×10 | 其他数据 | 含温度(偏移50: ~22.8°C) |
| 74-75 | 2B | — | 帧尾 | 固定或校验 |

### 欧拉角计算

从四元数 (w, x, y, z) 转换：

```python
import math

def quat_to_euler(qw, qx, qy, qz):
    """四元数转欧拉角 (rad)，返回 (roll, pitch, yaw)"""
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1, min(1, sinp)))

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
```

### 示例 Python 帧解析

```python
import serial, struct

PAYLOAD_OFFSET = 4
PAYLOAD_SIZE = 76

def parse_frame(frame):
    """解析 82 字节帧，返回传感器数据字典"""
    payload = frame[PAYLOAD_OFFSET:PAYLOAD_OFFSET + PAYLOAD_SIZE]

    qw = struct.unpack('<f', payload[18:22])[0]
    qx = struct.unpack('<f', payload[22:26])[0]
    qy = struct.unpack('<f', payload[26:30])[0]
    qz = struct.unpack('<f', payload[30:34])[0]

    # 温度 (偏移 50)
    temp = struct.unpack('<f', payload[50:54])[0]

    # 其他字段
    extra = {}
    for name, off in [('f34',34),('f38',38),('f42',42),('f46',46),
                       ('f54',54),('f58',58),('f62',62),('f66',66),('f70',70)]:
        extra[name] = struct.unpack('<f', payload[off:off+4])[0]

    return {'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz, 'temperature': temp, **extra}

def read_imu_frame(ser):
    """从串口流中读取一个完整的 IMU 帧"""
    buffer = bytearray()
    while True:
        chunk = ser.read(256)
        if chunk:
            buffer.extend(chunk)
        # 寻找帧头
        while len(buffer) >= 82:
            hdr = buffer.find(b'\x5a\xa5')
            if hdr < 0:
                buffer = buffer[-1:]
                break
            if hdr > 0:
                del buffer[:hdr]
            if len(buffer) < 82:
                break
            frame = bytes(buffer[:82])
            del buffer[:82]
            length = struct.unpack('<H', frame[2:4])[0]
            if length == 76:
                return frame

# 使用
ser = serial.Serial("COM12", 115200, timeout=0.5)
frame = read_imu_frame(ser)
data = parse_frame(frame)
```

### AT 命令支持

传感器支持 AT 命令（在数据流中穿插发送）：

| 命令 | 响应 |
|------|------|
| `AT+INFO\r\n` | 返回设备信息 (型号、固件版本等) |
| `AT+HELP\r\n` | 返回帮助信息 (经测试数据流干扰较大) |

---

## 3. 快速使用指南

```python
import serial, struct, time, math

# --- 力传感器 ---
def modbus_crc(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

force_ser = serial.Serial("COM11", 19200, timeout=0.3)

def read_force():
    req = bytes([1, 0x03, 0x0F, 0xA0, 0, 2])
    crc = modbus_crc(req)
    req += struct.pack('<H', crc)
    force_ser.reset_input_buffer()
    force_ser.write(req)
    time.sleep(0.06)
    resp = force_ser.read(256)
    if len(resp) >= 7 and struct.pack('<H', modbus_crc(resp[:-2])) == resp[-2:]:
        raw = struct.unpack('>H', resp[5:7])[0]
        return raw - 0x10000 if raw >= 0x8000 else raw
    return None

# --- IMU ---
imu_ser = serial.Serial("COM12", 115200, timeout=0.5)
imu_buf = bytearray()

def read_imu():
    global imu_buf
    imu_buf.extend(imu_ser.read(256))
    while len(imu_buf) >= 82:
        hdr = imu_buf.find(b'\x5a\xa5')
        if hdr < 0: imu_buf = imu_buf[-1:]; break
        if hdr > 0: del imu_buf[:hdr]
        if len(imu_buf) < 82: break
        frame = bytes(imu_buf[:82])
        del imu_buf[:82]
        if struct.unpack('<H', frame[2:4])[0] == 76:
            p = frame[4:80]
            qw = struct.unpack('<f', p[18:22])[0]
            qx = struct.unpack('<f', p[22:26])[0]
            qy = struct.unpack('<f', p[26:30])[0]
            qz = struct.unpack('<f', p[30:34])[0]
            return (qw, qx, qy, qz)
    return None

# 循环读取
while True:
    force = read_force()
    imu = read_imu()
    if force is not None and imu is not None:
        qw, qx, qy, qz = imu
        # 计算欧拉角...
        print(f"Force: {force/100:.4f}  Quat: {qw:.4f} {qx:.4f} {qy:.4f} {qz:.4f}")
    time.sleep(0.01)
```

---

## 注意事项

1. **力传感器**不会主动发送数据，必须通过 Modbus RTU 轮询读取
2. **IMU** 持续以 ~138 Hz 频率主动输出数据帧
3. 力传感器 Modbus 从站地址和波特率可通过 D056 仪表菜单修改
4. IMU 的 5A A5 协议与 FDILink 标准驱动中的 FC 帧头协议不同，可能是固件版本差异 (v1.6.8, 2025-07-17)
