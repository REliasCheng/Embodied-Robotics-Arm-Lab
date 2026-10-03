# System Architecture

## Scope

本仓库的可定位系统路径由 `so100.yaml`、LeRobot robot abstraction、control workflow 和两个 motor backend 组成。它描述的是 host-side Python integration，不是 MCU firmware architecture。

## Layer Model

```text
Input / Event
├── Leader arm joint state
└── Optional OpenCV camera frames
          ↓
Application / Core Logic
├── Teleoperate
├── Record
└── Replay
          ↓
Configuration / Abstraction
├── Hydra robot configuration
├── ManipulatorRobot
└── Motor bus interfaces
          ↓
Hardware Interface
├── GBot leader serial bus
└── Feetech follower serial bus
          ↓
Output
├── Follower joint and gripper commands
└── Optional local episode data
```

## Evidence Boundary

- `teleoperate`、`record` 与 `replay` mode 存在于 `lerobot/scripts/control_robot.py`。
- leader、follower 与 camera mapping 存在于 `lerobot/configs/robot/so100.yaml`。
- GBot 与 Feetech serial backend 存在于 `lerobot/common/robot_devices/motors/`。
- 公开材料没有证明实际设备已经连接、标定或完成运行。

Policy、dataset 与 training modules 是第三方框架源码的一部分，不作为本仓库已完成的 learned-control capability。
