#!/bin/zsh
set -e
cd -- "$(dirname -- "$0")"
viewer_python="$HOME/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3"
if [[ ! -x "$viewer_python" ]]; then
  print '找不到带 Tk 9 的 Python 运行环境，请重新配置后再启动。'
  exit 1
fi
exec "$viewer_python" -c '
import sys, runpy
from pathlib import Path
sys.path.append(str(Path.home()/".espressif/python_env/idf5.4_py3.9_env/lib/python3.9/site-packages"))
sys.argv = ["push_debug_viewer.py"] + sys.argv[1:]
runpy.run_path("push_debug_viewer.py", run_name="__main__")
' --port /dev/cu.usbserial-0001 --connect
