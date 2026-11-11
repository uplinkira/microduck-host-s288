"""读取交付配置，以独立参数调用 PyInstaller，支持含空格和中文的文件名。"""
import argparse
from pathlib import Path
import subprocess
import sys

from app_config import EXE_NAME


def main():
    parser = argparse.ArgumentParser(description='构建 Windows 单文件上位机')
    parser.add_argument('--name', default=EXE_NAME, help='EXE 文件名，不含 .exe；默认读取 app_config.py')
    args = parser.parse_args()
    name = args.name
    if (not name.strip() or name != name.strip() or name.endswith('.')
            or any(c in name for c in '<>:"/\\|?*')
            or any(ord(c) < 32 for c in name) or name.lower().endswith('.exe')):
        parser.error('名称应为合法 Windows 文件名，不含路径或 .exe 后缀')
    root = Path(__file__).resolve().parent
    result = subprocess.run([
        sys.executable, '-m', 'PyInstaller', '--noconfirm', '--clean',
        '--onefile', '--windowed', '--name', name,
        '--paths', str(root.parent / 'common'),
        '--paths', str(root.parent / 'rdk_linux'),
        '--hidden-import', 'serial', '--hidden-import', 'serial.tools.list_ports',
        '--hidden-import', 'ruckig', '--copy-metadata', 'ruckig',
        str(root / 'pose_ui.py'),
    ], cwd=root)
    if result.returncode == 0:
        print(f'Built: {root / "dist" / (name + ".exe")}', flush=True)
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
