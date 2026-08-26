"""Interactive ESP32-S3 controller that also records serial telemetry."""

from __future__ import annotations

import argparse
from datetime import datetime
import msvcrt
from pathlib import Path
import sys
import time

import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Drive the car from the keyboard while saving all telemetry."
    )
    parser.add_argument("--port", default="COM5", help="serial port (default: COM5)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--wheel-test",
        action="store_true",
        help="show three-wheel diagnostic controls and use a wheel-test log name",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="log path; defaults to telemetry_logs/manual-<timestamp>.log",
    )
    return parser.parse_args()


def default_output_path(wheel_test: bool = False) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    prefix = "wheel-test" if wheel_test else "manual"
    return Path(__file__).resolve().parent / "telemetry_logs" / (
        f"{prefix}-{timestamp}.log"
    )


def main() -> int:
    args = parse_args()
    output_path = (args.output or default_output_path(args.wheel_test)).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Opening {args.port} at {args.baud} baud")
    if args.wheel_test:
        print("THREE-WHEEL TEST: 1=A/E1, 2=B/E2, 4=D/E4, T=all")
        print("Lift the chassis before testing; X/SPACE is emergency stop")
    else:
        print("W/S drive, A/D turn, Q/E strafe, F line follow, X/SPACE stop")
    print("Press Ctrl+] to stop the car, close the port, and finish recording")
    print(f"Recording to: {output_path}")

    try:
        port = serial.Serial(
            args.port,
            args.baud,
            timeout=0.02,
            write_timeout=1.0,
            rtscts=False,
            xonxoff=False,
        )
    except serial.SerialException as exc:
        print(f"Unable to open {args.port}: {exc}", file=sys.stderr)
        return 2

    port.dtr = False
    port.rts = False

    try:
        with output_path.open("w", encoding="utf-8", newline="") as log:
            while True:
                if msvcrt.kbhit():
                    key = msvcrt.getwch()
                    if key == "\x1d":  # Ctrl+]
                        break
                    if key in ("\x00", "\xe0"):
                        if msvcrt.kbhit():
                            msvcrt.getwch()
                        continue
                    try:
                        port.write(key.encode("utf-8"))
                    except serial.SerialTimeoutException:
                        print("\nSerial write timeout; sending emergency stop")
                        break

                waiting = port.in_waiting
                data = port.read(waiting if waiting else 1)
                if data:
                    text = data.decode("utf-8", errors="replace")
                    sys.stdout.write(text)
                    sys.stdout.flush()
                    log.write(text)
                    log.flush()
                else:
                    time.sleep(0.002)
    except KeyboardInterrupt:
        print("\nInterrupted; sending emergency stop")
    finally:
        try:
            port.write(b"x")
            port.flush()
            time.sleep(0.05)
        except serial.SerialException:
            pass
        port.close()

    print(f"\nTelemetry saved to: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
