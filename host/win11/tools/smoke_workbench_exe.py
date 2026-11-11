"""原生 Windows EXE 离线冒烟；仅控制本脚本启动的 --sim 进程。

测试辅助依赖 pywinauto/psutil 不属于生产运行依赖。需交互式桌面。
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

HOST_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_DIR))
from app_config import APP_VERSION, EXE_NAME, WINDOW_TITLE

import psutil
from pywinauto import Desktop
import win32con
import win32gui
import win32process


def wait_for(function, timeout=25):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = function()
        if value:
            return value
        time.sleep(.2)
    raise TimeoutError('等待原生窗口/控件超时')


def check(exe, folder, title=None, workbench=False, hidden=False):
    expected = title or WINDOW_TITLE
    startup = None
    if hidden:
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = win32con.SW_HIDE
    process = subprocess.Popen([str(exe), '--sim'] + (['--title', title] if title else []),
                               cwd=exe.parent, startupinfo=startup, creationflags=subprocess.CREATE_NO_WINDOW)
    root = psutil.Process(process.pid)
    owned = {process.pid: root.create_time()}
    desktop = Desktop(backend='uia')
    record = dict(version=APP_VERSION, expected_title=expected, process_id=process.pid, sim=True, hidden=hidden)

    def windows():
        try:
            for child in root.children(recursive=True):
                owned[child.pid] = child.create_time()
        except psutil.NoSuchProcess:
            pass
        handles = []
        def collect(hwnd, _):
            _, pid = win32process.GetWindowThreadProcessId(hwnd)
            if pid in owned and win32gui.GetWindowText(hwnd):
                if hidden:
                    win32gui.ShowWindow(hwnd, win32con.SW_HIDE)
                handles.append(hwnd)
        win32gui.EnumWindows(collect, None)
        top = [desktop.window(handle=hwnd).wrapper_object() for hwnd in handles]
        nested = [child for window in top for child in window.descendants(control_type='Window')]
        return top + nested

    try:
        main = wait_for(lambda: next((w for w in windows() if w.window_text() == expected), None))
        record['actual_title'] = main.window_text()
        if workbench:
            desktop.window(handle=main.handle).child_window(title='离线工作台', control_type='Button').invoke()
            print('Owned windows:', [w.window_text() for w in windows()], flush=True)
            dialog = wait_for(lambda: next((w for w in windows() if w.window_text() == '构型与平顺轨迹工作台 · 离线'), None))
            spec = desktop.window(handle=dialog.handle)
            tab = spec.child_window(title='平顺轨迹预览', control_type='TabItem')
            tab.select()
            wait_for(lambda: tab.is_selected())
            spec.child_window(title='载入离线示例', control_type='Button').invoke()
            wait_for(lambda: spec.child_window(title='人工离线示例 · 非实机验证动作', control_type='Text').exists(timeout=0))
            spec.child_window(title='生成平顺轨迹', control_type='Button').invoke()
            export = spec.child_window(title='导出参考 JSON / CSV', control_type='Button')
            wait_for(lambda: export.is_enabled())
            record['trajectory_generated'] = True
            record['visible_text'] = [w.window_text() for w in dialog.descendants()
                                      if w.window_text() and w.element_info.control_type in ('Text', 'Edit')]
            screenshot = folder/'workbench_exe.png'
            if not hidden:
                dialog.capture_as_image().save(screenshot)
                record['screenshot'] = str(screenshot)
            win32gui.PostMessage(dialog.handle, win32con.WM_CLOSE, 0, 0)
            wait_for(lambda: not win32gui.IsWindow(dialog.handle))
        win32gui.PostMessage(main.handle, win32con.WM_CLOSE, 0, 0)
        process.wait(timeout=20)
        record['exit_code'] = process.returncode
        record['normal_wm_close'] = True
    finally:
        if 'dialog' in locals() and win32gui.IsWindow(dialog.handle):
            record['diagnostic_text'] = [w.window_text() for w in dialog.descendants()
                                         if w.window_text() and w.element_info.control_type in ('Text', 'Edit')]
        survivors = []
        for pid, created in owned.items():
            try:
                item = psutil.Process(pid)
                if item.create_time() == created and item.is_running():
                    survivors.append(item)
            except psutil.NoSuchProcess:
                pass
        record['forced_cleanup_pids'] = [item.pid for item in survivors]
        for item in survivors:
            item.terminate()
        psutil.wait_procs(survivors, timeout=5)
        print(json.dumps(record, ensure_ascii=False), flush=True)
    if record['forced_cleanup_pids']:
        raise RuntimeError('测试进程需要强制清理，不能视为正常退出')
    return record


def main():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exe', type=Path, default=HOST_DIR/'dist'/f'{EXE_NAME}.exe')
    parser.add_argument('--hidden', action='store_true', help='隐藏本脚本启动的测试窗口')
    args = parser.parse_args()
    folder = Path(__file__).resolve().parents[1]/'build'
    folder.mkdir(exist_ok=True)
    records = [check(args.exe.resolve(), folder, workbench=True, hidden=args.hidden),
               check(args.exe.resolve(), folder, title=f'离线验收_{APP_VERSION}', hidden=args.hidden)]
    (folder/'smoke_workbench_exe.json').write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
