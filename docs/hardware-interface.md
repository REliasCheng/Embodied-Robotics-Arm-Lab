# Hardware Interface

## Configured Robot Layout

`so100.yaml` 描述一组 leader arm 和一组 follower arm。每组配置包含：

- `shoulder_pan`
- `shoulder_lift`
- `elbow_flex`
- `wrist_flex`
- `wrist_roll`
- `gripper`

因此当前公开配置可描述为 five motion joints plus one gripper channel，不表述为经过测量确认的 six-degree-of-freedom system。

## Motor Buses

| Side | Backend | Evidence |
| --- | --- | --- |
| Leader | `GBotMotorsBus` | `so100.yaml` 与 `gbot.py` |
| Follower | `FeetechMotorsBus` | `so100.yaml` 与 `feetech.py` |

GBot backend 源码设置 `230400` baud rate。Follower backend 的协议和 motor model 由 Feetech implementation 与 YAML mapping 决定。USB serial port name 必须按本机设备枚举结果配置，不能直接沿用示例端口。

## Camera Interfaces

配置包含两个 `OpenCVCamera` entry，默认 index 为 `0` 与 `2`。这些值只表示配置样例，不能证明相机存在或图像采集已经成功。

## Mechanical and Safety Boundary

公开内容没有提供可复核的 physical joint range、collision model、emergency stop、workspace measurement 或 load limit。运行前必须依据实际机械结构和电机规格重新确认这些限制。
