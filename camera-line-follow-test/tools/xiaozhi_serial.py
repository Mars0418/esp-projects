"""Send one explicit XiaoZhi UART command and read a bounded log window."""
import argparse
import re
import time
import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["bind", "connect", "listen", "send", "stop", "status"])
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--seconds", type=int, default=30)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error("--seconds must be 1..60")
    port = serial.Serial(baudrate=115200, timeout=0.2, write_timeout=2)
    port.dtr = False
    port.rts = False
    port.port = args.port
    if args.command == "listen":
        print("Microphone audio will be uploaded to XiaoZhi for up to 15 seconds. Ctrl+C aborts.", flush=True)
    try:
        port.open()
        port.reset_input_buffer()
        port.write(("xz " + args.command + "\n").encode("ascii"))
        port.flush()
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            line = port.readline().decode("utf-8", errors="replace")
            line = re.sub(r"\x1b\[[0-9;]*m", "", line).strip()
            if line and any(tag in line for tag in ("XIAOZHI", "CAR_AUDIO", "esp-tls", "HTTP_CLIENT", "Guru", "assert", "mbedtls", "wifi:connected", "LAN_URL", "certificate")):
                print(line, flush=True)
    except KeyboardInterrupt:
        if port.is_open:
            port.write(b"xz stop\n")
            port.flush()
        print("Stop requested", flush=True)
    finally:
        port.close()


if __name__ == "__main__":
    main()
