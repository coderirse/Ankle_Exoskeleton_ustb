# HiPNUC HI91 → Foxglove 可视化

## 原因说明

若只创建 `Channel("/hi91", message_encoding="json")` 且**不注册 Schema**，Foxglove 侧只会看到话题名 `/hi91`，**不会出现** `acc`、`gyr` 等字段供 Plot 选择。  
本程序已为 `/hi91` 注册完整 JSON Schema，并为各物理量单独发布子话题（Plot 可直接选 `x/y/z`）。

## 发布话题一览

| 话题 | 类型 | HI91 字段 | 单位 |
|------|------|-----------|------|
| `/hi91` | JSON（带 Schema） | 整帧 | 见下表 |
| `/hi91/acc` | Vector3 | 加速度 | m/s² |
| `/hi91/gyr` | Vector3 | 角速度 | deg/s |
| `/hi91/mag` | Vector3 | 磁力计 | uT |
| `/hi91/quat` | Quaternion | 四元数 | w,x,y,z |
| `/hi91/euler` | JSON | pitch, roll, yaw | deg |
| `/hi91/system_time` | JSON | 时间戳 | ms |
| `/hi91/temperature` | JSON | 温度 | °C |
| `/hi91/air_pressure` | JSON | 气压 | Pa |
| `/hi91/pps_sync_stamp` | JSON | PPS 同步 | ms |
| `/hi91/attitude` | FrameTransforms | 姿态（3D 用） | — |

### `/hi91` 整帧 JSON 字段

与 `hihost read` 相同：`type`, `system_time`, `acc`, `gyr`, `mag`, `pitch`, `roll`, `yaw`, `quat`, `air_pressure`，另含 `pps_sync_stamp`, `temperature`。

## 使用

```bash
sudo ./run.sh -p /dev/ttyACM0 -b 115200
```

- **终端**：默认与 `hihost read` 相同，滚动显示 HI91 JSON + `Frame Rate: xx fps`
- **Foxglove**：`ws://localhost:8765`
- 仅 Foxglove、不刷终端：`sudo ./run.sh -q`

**注意：** 不要同时运行 `hihost read` 与本程序（会抢占同一串口）。

### Plot 面板怎么选

- 欧拉角：话题 `/hi91/euler`，字段 `pitch` / `roll` / `yaw`
- 加速度：话题 `/hi91/acc`，字段 `x` / `y` / `z`
- 角速度：话题 `/hi91/gyr`，字段 `x` / `y` / `z`
- 或整帧：话题 `/hi91`，字段 `acc[0]` 等（需 Schema 已注册）

### 3D 面板

订阅 `/hi91/attitude`
