"""Save one full-resolution JPEG from the firmware's CALIB UART mode."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import serial


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()

    port = serial.Serial(args.port, 115200, timeout=0.1)
    try:
        # Opening the CP210x console toggles reset on this board.  Wait until
        # USB camera startup has completed before sending the mode command.
        time.sleep(3.0)
        port.reset_input_buffer()
        port.write(b"CALIB,1\n")
        port.flush()
        time.sleep(0.2)
        port.baudrate = 921600

        buffer = bytearray()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            buffer.extend(port.read(4096))
            start = buffer.find(b"@CALJPEG,")
            if start < 0:
                if len(buffer) > 4096:
                    del buffer[:-256]
                continue
            newline = buffer.find(b"\n", start)
            if newline < 0:
                continue
            fields = buffer[start:newline].decode("ascii").split(",")
            if len(fields) != 5 or fields[0] != "@CALJPEG":
                del buffer[:newline + 1]
                continue
            sequence, width, height, payload_size = map(int, fields[1:])
            payload = bytearray(buffer[newline + 1:])
            while len(payload) < payload_size and time.monotonic() < deadline:
                payload.extend(port.read(payload_size - len(payload)))
            if len(payload) >= payload_size:
                jpeg = bytes(payload[:payload_size])
                if not (jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9")):
                    raise ValueError("Captured frame is not a complete JPEG.")
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_bytes(jpeg)
                print(f"saved={args.output} sequence={sequence} size={width}x{height} bytes={payload_size}")
                return
        raise TimeoutError("No complete calibration JPEG arrived from the camera.")
    finally:
        try:
            port.write(b"CALIB,0\n")
            port.flush()
            time.sleep(0.15)
        finally:
            port.close()


if __name__ == "__main__":
    main()
