# Development Environment

## Source Context

第三方 package metadata 指定 Python `>=3.10,<3.13`，并使用 Poetry build backend。依赖声明位于：

- `third_party/genkiarm/pyproject.toml`
- `third_party/genkiarm/requirements.txt`

## Inspection Entry

```powershell
cd third_party/genkiarm
python -m build --no-isolation
```

该命令只验证 package metadata 和 source packaging。它不会连接机械臂，也不会验证 serial、camera、dataset 或 policy runtime。

## Runtime Prerequisites

运行 control workflow 前至少需要：

- 与 package metadata 匹配的 Python environment
- 根据实际硬件选择并安装 serial / motor extras
- 枚举并确认 leader 与 follower serial ports
- 如需 camera input，确认 OpenCV camera index
- 按实际设备完成 calibration

不要在未确认设备、机械范围和紧急停止方式时发送运动命令。
