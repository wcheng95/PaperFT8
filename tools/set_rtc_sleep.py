#!/usr/bin/env python3
"""Set Mini-FT8 RTC to current UTC time, then enter deep sleep."""

import sys
import time
import serial
import serial.tools.list_ports
from datetime import datetime, timezone


def find_port():
    """Auto-detect the USB serial port."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        # ESP32-S3 USB Serial JTAG
        if "jtag" in desc or "303a:1001" in hwid:
            return p.device
    return None


def send_cmd(ser, cmd):
    """Send a command and return the response line."""
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    # Read lines until we get a non-prompt, non-empty response
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line and not line.startswith("MINIFT8>"):
            return line
    return ""


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("Usage: set_rtc_sleep.py [PORT]")
        print("No ESP32-S3 USB JTAG port detected.")
        sys.exit(1)

    print(f"Port: {port}")

    now = datetime.now(timezone.utc)
    date_str = now.strftime("%Y-%m-%d")

    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.1)

    # Set date first (no timing sensitivity)
    send_cmd(ser, f"DATE {date_str}")

    # Wait for the next second boundary, then send TIME for that second
    now = datetime.now(timezone.utc)
    frac = now.microsecond / 1_000_000
    time.sleep(1.0 - frac)
    target = datetime.now(timezone.utc)
    time_str = target.strftime("%H:%M:%S")
    # Date may have rolled over at midnight
    date_str2 = target.strftime("%Y-%m-%d")
    if date_str2 != date_str:
        send_cmd(ser, f"DATE {date_str2}")
        date_str = date_str2

    send_cmd(ser, f"TIME {time_str}")
    print(f"RTC set to {date_str} {time_str} UTC")

    # Send SLEEP and close immediately — device won't respond
    ser.write(b"SLEEP\r\n")
    ser.flush()
    time.sleep(0.1)
    ser.close()
    print("SLEEP sent")


if __name__ == "__main__":
    main()
