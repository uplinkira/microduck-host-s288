# Microduck Host Unitree S288

宇树 S288 舵机上位机，提供 Windows 图形界面，支持串口调试、标零、位姿录制与播放。

## 功能

- 串口选择、连接与舵机状态反馈。
- 电脑端零位保存、方向设置、目标角度、速度和增益调节。
- 位姿录制、编辑、顺序播放，支持 JSON 导入保存和 CSV 导出。
- 遥测记录、实测采集与统计。
- 离线构型编辑、MJCF 静态检查、平顺轨迹预览与参考导出。
- 离线演示模式，可在没有舵机时操作界面。

## 下载使用

从 [Releases](https://github.com/uplinkira/microduck-host-unitree-s288/releases/latest) 下载 Windows x64 压缩包，解压后运行 `microduck-host-unitree-s288.exe`。

- 实物连接：选择串口后点击连接，默认波特率 `6000000`。
- 离线演示：运行 `microduck-host-unitree-s288.exe --sim`。

## 源码运行

```powershell
python -m pip install -r host/win11/requirements.txt
python host/win11/pose_ui.py
```

离线演示：

```powershell
python host/win11/pose_ui.py --sim
```

## 使用说明

- 零位与位姿保存在程序旁边的 JSON 文件中，换电脑时一并迁移。
- 停止播放保持最后指令位置；急停请求舵机卸力。
- 参数、数据文件和构建方式见 [Windows 使用说明](host/win11/README.md)。
