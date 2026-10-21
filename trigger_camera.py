"""Fire the camera trigger once via the camera_trigger.ino sketch.

Usage:
    python trigger_camera.py                  # auto-detect the Arduino port
    python trigger_camera.py --port /dev/cu.usbmodem14101
    python trigger_camera.py --pulse-ms 100   # set pulse width before firing

Requires: pip install pyserial
"""

import argparse
import sys
import time

import serial
from serial.tools import list_ports

BAUD = 115200
READY_TIMEOUT_S = 3.0  # Uno/Nano reset when the port opens; bootloader + setup() takes ~2 s
REPLY_TIMEOUT_S = 1.0


def find_port():
    """Return the first port that looks like an Arduino, or None."""
    for p in list_ports.comports():
        desc = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
        if any(k in desc for k in ("usbmodem", "usbserial", "arduino", "ch340", "wch")):
            return p.device
    return None


def read_line(ser, timeout):
    """Read one non-empty line, or return None on timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            return line
    return None


def send(ser, cmd):
    """Send a command and return the board's reply line."""
    ser.write((cmd + "\n").encode())
    ser.flush()
    reply = read_line(ser, REPLY_TIMEOUT_S)
    print(f"> {cmd}\n< {reply}")
    return reply


def main():
    parser = argparse.ArgumentParser(description="Trigger the camera once.")
    parser.add_argument("--port", help="serial port (default: auto-detect)")
    parser.add_argument("--pulse-ms", type=int, help="pulse width in ms (1-5000)")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No Arduino found. Pass --port (see: python -m serial.tools.list_ports).")

    with serial.Serial(port, BAUD, timeout=0.1) as ser:
        # Opening the port resets the board, which also re-arms it (fired=false).
        ready = read_line(ser, READY_TIMEOUT_S)
        if ready != "READY":
            print(f"Warning: no READY from board (got {ready!r}); continuing anyway.")
        ser.reset_input_buffer()

        if args.pulse_ms is not None:
            if send(ser, f"P{args.pulse_ms}") != "OK":
                sys.exit("Failed to set pulse width.")

        reply = send(ser, "T")
        if reply == "OK":
            print("Camera triggered.")
        elif reply and "already fired" in reply:
            sys.exit("Board already fired and was not reset. Reset the board or send R to re-arm.")
        else:
            sys.exit(f"Trigger failed: {reply!r}")

        # Keep the port open until the pulse ends: closing it can toggle DTR and
        # reset the board, which would cut the pulse short.
        time.sleep((args.pulse_ms or 50) / 1000 + 0.05)


if __name__ == "__main__":
    main()
