# 小车上位机 V1

`host_app` 是基于 `Python + PySide6` 的 Windows 优先桌面上位机，用于连接当前 STM32 固件并完成：

- 串口连接与断线重连
- 文本兼容模式与结构化 telemetry 模式切换
- 手动控制、LiDAR 启停、导航目标下发
- 运行状态、地图、路径、点云和日志可视化
- 原始串口录包与离线回放

## 安装

```powershell
cd host_app
python -m venv .venv
. .venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## 运行

```powershell
cd host_app
python main.py
```

## 打包

```powershell
cd host_app
.\build.ps1
```
