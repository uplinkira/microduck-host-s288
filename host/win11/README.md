# Windows 使用说明

## 启动

- Windows x64 发布包：解压后运行 `microduck-host-unitree-s288.exe`。
- 主窗口刷新、选择串口后连接，默认波特率 `6000000`。
- 先标零，再使能并从小行程开始调试。机械行程由操作者核对，界面不施加机械角度限位。
- 停止播放保持最后指令；急停/全部卸力请求释放力矩。

源码运行，在仓库根目录执行：

```powershell
python -m pip install -r host/win11/requirements.txt
python host/win11/pose_ui.py
```

## 参数

| 参数 | 作用 |
|---|---|
| `--sim` | 离线演示 |
| `--port COM5` | 指定串口并在启动后连接 |
| `--baud 6000000` | 指定波特率 |
| `--ids 7,8,14` | 指定本次关节，地址为 `0–14` 且不重复 |
| `--list-ports` | 列出串口后退出 |
| `--title` | 覆盖窗口标题 |
| `--no-watchdog` | 关闭默认超时保护 |

## 数据

| 文件 | 内容 |
|---|---|
| `calib_host_xianduck.json` | 电脑端零位与方向 |
| `poses_xianduck.json` | 位姿库 |
| `poses_xianduck.csv` | 导出的位姿表 |
| `log_时间.csv` | 遥测记录 |
| `s288_时间_随机后缀/` | 实测会话 |

- 源码运行时，默认数据在入口目录；EXE 运行时在程序旁边。
- 位姿逐帧设置运动速度与到位后的暂停；支持 JSON 导入保存和 CSV 导出。
- 改零位后重新核对原位姿。换电脑时复制匹配的零位与位姿文件。

## 离线工作台

- 从主窗口打开，可编辑构型、检查 MJCF、预览平顺轨迹并导出 JSON/CSV 参考。
- 每个关键帧停稳；参考导出与实物播放分别操作。

## 构建与检查

```powershell
.\host\win11\build.bat
```

产物为 `host/win11/dist/microduck-host-unitree-s288.exe`。

```powershell
$env:QT_QPA_PLATFORM = 'offscreen'
python -m unittest discover -s host/win11/tests -v
python host/common/motor_sweep.py --selftest
```
