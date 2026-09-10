"""Run ESP-IDF with a complete child-process environment on Windows."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

p = argparse.ArgumentParser()
p.add_argument('--idf', required=True)
p.add_argument('--tools', required=True)
p.add_argument('--build', required=True)
p.add_argument('--action', choices=('build', 'flash', 'monitor'), default='build')
p.add_argument('--port', default='')
a = p.parse_args()
project = Path(__file__).resolve().parents[1]
tools = Path(a.tools).resolve()
idf = Path(a.idf).resolve()
build = Path(a.build).resolve()
if not str(build).isascii():
    p.error('Use an ASCII build directory for the Windows toolchain.')
if a.action != 'build' and not a.port:
    p.error('Specify the actual connected port.')
env = os.environ.copy()
bins = [str(Path(sys.executable).parent)]
for pattern in ('cmake/*/bin', 'ninja/*', 'xtensa-esp-elf/*/xtensa-esp-elf/bin',
                'esp32ulp-elf/*/esp32ulp-elf/bin', 'riscv32-esp-elf/*/riscv32-esp-elf/bin'):
    matches = sorted(tools.glob(pattern))
    if matches:
        bins.append(str(matches[-1]))
env['PATH'] = os.pathsep.join(bins + [env.get('PATH', '')])
env['IDF_PATH'] = str(idf)
env['IDF_TOOLS_PATH'] = str(tools)
env['IDF_PYTHON_ENV_PATH'] = str(Path(sys.executable).parent.parent)
env['IDF_CCACHE_ENABLE'] = '0'
env['PYTHONUTF8'] = '1'
roms = sorted(tools.glob('esp-rom-elfs/*'))
if roms:
    env['ESP_ROM_ELF_DIR'] = str(roms[-1])
index = int(env.get('GIT_CONFIG_COUNT', '0'))
env[f'GIT_CONFIG_KEY_{index}'] = 'safe.directory'
env[f'GIT_CONFIG_VALUE_{index}'] = str(idf)
env['GIT_CONFIG_COUNT'] = str(index + 1)
ninja = shutil.which('ninja', path=env['PATH'])
compiler = shutil.which('xtensa-esp32s3-elf-gcc', path=env['PATH'])
if not ninja or not compiler:
    p.error('Missing Ninja or ESP32-S3 compiler under --tools.')
cmd = [sys.executable, str(idf / 'tools/idf.py'), '-B', str(build),
       '-D', f'CMAKE_MAKE_PROGRAM={ninja}']
if a.action == 'build':
    cmd += ['build']
else:
    cmd += ['-p', a.port, 'app-flash' if a.action == 'flash' else 'monitor']
result = subprocess.run(cmd, cwd=project, env=env)
if result.returncode:
    sys.exit(result.returncode)
if a.action == 'build':
    output = project / 'firmware'
    output.mkdir(exist_ok=True)
    for suffix in ('.bin', '.elf', '.map'):
        shutil.copy2(build / ('camera-line-follow-test' + suffix), output)

