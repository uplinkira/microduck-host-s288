# 上位机代码

| 目录 | 内容 |
|---|---|
| [win11/](win11/) | Windows 界面、标零、位姿、测量与离线工作台 |
| [common/](common/) | S288 总线编解码与收发 |
| [rdk_linux/](rdk_linux/) | Tkinter 调试界面与共享控制基础 |

在仓库根目录运行：

```powershell
python -m pip install -r host/win11/requirements.txt
python host/win11/pose_ui.py --sim
```
