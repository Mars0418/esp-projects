"""Bounded USB-audio check over UART; never records/uploads audio or moves motors."""
import argparse
import re
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--tone", action="store_true", help="also play one quiet 200 ms tone")
    args = parser.parse_args()
    port = serial.Serial(baudrate=115200, timeout=0.2, write_timeout=2)
    # Do not intentionally reset the board or enter the bootloader.
    port.dtr = False
    port.rts = False
    port.port = args.port
    collected = []

    def collect(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            line = port.readline().decode("utf-8", errors="replace")
            line = re.sub(r"\x1b\[[0-9;]*m", "", line).strip()
            if any(tag in line for tag in ("CAR_AUDIO", "USB_AV_PROBE", "Guru Meditation", "assert failed")):
                print(line, flush=True)
                collected.append(line)

    def send(command):
        port.write((command + "\n").encode("ascii"))
        port.flush()

    try:
        port.open()
        collect(1)
        send("audio status")
        collect(3)
        if not any("STATUS mic_ready=1" in s and "speaker_ready=1" in s for s in collected):
            raise RuntimeError("USB microphone/speaker are not both ready; inspect startup logs")
        send("audio mic")
        collect(5)
        if not any(re.search(r"MIC_METER samples=[1-9]\d*", s) for s in collected):
            raise RuntimeError("No microphone PCM received")
        if args.tone:
            send("audio tone")
            collect(2)
            if not any("TONE_QUEUED result=ESP_OK suspend=ESP_OK" in s for s in collected):
                raise RuntimeError("Speaker test did not complete")
        send("audio stop")
        collect(1)
        send("audio status")
        collect(1)
        statuses = [s for s in collected if "STATUS mic_ready=" in s]
        if not statuses or not all(token in statuses[-1] for token in
                ("mic_active=0", "mic_errors=0", "speaker_errors=0")):
            raise RuntimeError("Stream errors or microphone did not stop")
        print("PASS: UART control, microphone PCM, clean stop" +
              ("; tone queued (audibility needs human confirmation)" if args.tone else ""))
    finally:
        if port.is_open:
            try:
                send("audio stop")
            finally:
                port.close()


if __name__ == "__main__":
    main()
