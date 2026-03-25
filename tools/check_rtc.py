#!/usr/bin/env python3
"""Compare Mini-FT8 RTC with host UTC clock by detecting second transition."""

import sys
import time
import serial
import serial.tools.list_ports
from datetime import datetime, timezone


def find_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "jtag" in desc or "303a:1001" in hwid:
            return p.device
    return None


def read_time(ser):
    """Send TIME command and return the response string."""
    ser.reset_input_buffer()
    ser.write(b"TIME\r\n")
    ser.flush()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("TIME "):
            return line[5:]
    return ""


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("Usage: check_rtc.py [PORT]")
        sys.exit(1)

    ser = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(0.1)

    # Get initial reading
    prev = read_time(ser)
    if not prev:
        print("Failed to read device clock")
        ser.close()
        sys.exit(1)

    # Poll until the second ticks over — that's the precise boundary
    for _ in range(200):  # ~10s max
        cur = read_time(ser)
        if cur and cur != prev:
            host_now = datetime.now(timezone.utc)
            break
    else:
        print("Timeout waiting for second transition")
        ser.close()
        sys.exit(1)

    # Also read DATE
    ser.reset_input_buffer()
    ser.write(b"DATE\r\n")
    ser.flush()
    dev_date = ""
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("DATE "):
            dev_date = line[5:]
            break
    ser.close()

    dev_str = f"{dev_date} {cur}"
    host_str = host_now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    # Device just ticked to `cur`, so device time is cur.000
    from datetime import datetime as dt
    dev_dt = dt.strptime(dev_str, "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
    diff = (host_now - dev_dt).total_seconds()

    print(f"Device: {dev_str}.000")
    print(f"Host:   {host_str}")
    print(f"Diff:   {diff:+.3f}s (positive = device behind)")


if __name__ == "__main__":
    main()
