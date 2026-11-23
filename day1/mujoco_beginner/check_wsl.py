"""WSL2 실습 환경을 기록한다. GUI 지원 여부는 실제 viewer 실행으로 확인한다."""
import json
import os
import platform
import sys
from pathlib import Path

info = {
    "system": platform.system(),
    "kernel": platform.release(),
    "python": sys.version,
    "executable": sys.executable,
    "working_directory": str(Path.cwd()),
    "os_release": Path('/etc/os-release').read_text() if Path('/etc/os-release').exists() else None,
    "DISPLAY": os.environ.get('DISPLAY'),
    "WAYLAND_DISPLAY": os.environ.get('WAYLAND_DISPLAY'),
    "XDG_RUNTIME_DIR": os.environ.get('XDG_RUNTIME_DIR'),
    "MUJOCO_GL": os.environ.get('MUJOCO_GL'),
    "wslg_directory": Path('/mnt/wslg').exists(),
}
print(json.dumps(info, ensure_ascii=False, indent=2))
if info['system'] != 'Linux' or 'WSL2' not in info['kernel'].upper():
    raise SystemExit('Expected WSL2 Linux. Open Ubuntu-22.04 and run: uv run check_wsl.py')
out = Path(__file__).resolve().parent / 'results'
out.mkdir(exist_ok=True)
(out / 'wsl_environment.json').write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding='utf-8')
