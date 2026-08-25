"""Run esptool with smaller ESP32-S3 stub flash blocks.

This is a local recovery wrapper for USB Serial/JTAG links that disconnect
after a default 16 KiB flash block. It does not modify the firmware image.
"""

from esptool import _main
from esptool.targets.esp32s3 import ESP32S3StubLoader


ESP32S3StubLoader.FLASH_WRITE_SIZE = 0x1000


if __name__ == "__main__":
    _main()
