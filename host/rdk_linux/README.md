# Linux / Tkinter 调试入口

提供 S288 关节探测、标零、目标角度和限位调节。

依赖 Python 3、tkinter；真实串口需要 pyserial 和串口访问权限。

在仓库根目录运行：

```bash
python3 host/rdk_linux/servo_ui.py --sim
python3 host/rdk_linux/servo_ui.py --port /dev/ttyACM0 --ids 7,8,14
```

- 波特率默认 `6000000`，已定义关节位于 `KNOWN` 表。
- `ui.sh` 支持 Mac + XQuartz + SSH X11 启动；需先配置自己的 SSH 别名和板端环境。
- 脚本默认 SSH 别名为 `rdk`，串口为 `/dev/ttyACM0`。
