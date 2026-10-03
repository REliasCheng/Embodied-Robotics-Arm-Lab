# Control Flow

## Teleoperation

1. `control_robot.py` 解析 `teleoperate` mode 与 robot configuration。
2. Hydra 根据 `so100.yaml` 创建 `ManipulatorRobot`。
3. leader motor bus 提供关节与夹爪输入。
4. control loop 将输入转换为 follower command。
5. follower motor bus 发送目标值。

## Recording

`record` mode 在 control loop 外增加 episode lifecycle、local dataset path 与 optional camera frames。源码入口存在，但公开仓库没有附带 dataset、capture image、video 或 recording log。

## Replay

`replay` mode 从本地 episode dataset 读取 action sequence 并调用 robot action interface。公开仓库未提供可复核的 replay result。

## Reference-Only Paths

第三方源码保留 training、evaluation 与多种 policy module，以维持来源结构和接口可读性。未提供对应 model weights、training logs、metrics 或 hardware execution evidence，因此这些路径不被列为已验证功能。
