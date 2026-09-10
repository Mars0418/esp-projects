"""Visible UART wake monitor. Ctrl+C releases the port without disabling wake."""
import argparse
import datetime
import re
from pathlib import Path
import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM9')
    parser.add_argument('--connect', action='store_true', help='Connect cloud session without starting microphone upload')
    args = parser.parse_args()
    log_path = Path(__file__).resolve().parents[1] / 'captures' / 'wake-monitor.log'
    log_path.parent.mkdir(exist_ok=True)
    port = serial.Serial(baudrate=115200, timeout=0.3, write_timeout=2)
    port.dtr = False
    port.rts = False
    port.port = args.port
    try:
        port.open()
        print(f'Listening on {args.port}. Log: {log_path}', flush=True)
        print('Say: 你好乐迪. Wait for LISTENING before asking a question.', flush=True)
        with log_path.open('a', encoding='utf-8') as log:
            log.write(f'\n--- {datetime.datetime.now().isoformat()} ---\n')
            port.write(b'xz status\n')
            if args.connect:
                port.write(b'xz connect\n')
            while True:
                line = port.readline().decode('utf-8', errors='replace').strip()
                line = re.sub(r'\x1b\[[0-9;]*m', '', line)
                line = re.sub(r'AP_PASSWORD=\S+', 'AP_PASSWORD=[redacted]', line)
                if line and any(tag in line for tag in ('WAKE', 'XIAOZHI', 'CAR_AUDIO', 'WIFI_DEBUG', 'Guru', 'assert', 'esp-tls', 'HTTP_CLIENT')):
                    print(line, flush=True)
                    log.write(line + '\n')
                    log.flush()
    except KeyboardInterrupt:
        print('\nMonitor closed; device wake remains enabled.')
    finally:
        port.close()


if __name__ == '__main__':
    main()
