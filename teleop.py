#!/usr/bin/env python3
"""Keyboard teleop for the two-motor Navibot firmware over USB serial."""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required. Install it with: python3 -m pip install pyserial") from exc


SYNC = 0x5A
VERSION = 0x01
TARGET_NODES = (0x01, 0x02)
MSG_REQUEST = 0x01
MSG_REPLY = 0x81
BAUD = 115200
HEARTBEAT_SECONDS = 0.10
REPLY_WAIT_SECONDS = 0.04


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_command(node_id: int, sequence: int, left_rpm: int, right_rpm: int) -> bytes:
    payload = struct.pack("<hh", left_rpm, right_rpm)
    header_and_payload = bytes((VERSION, node_id, MSG_REQUEST, sequence, len(payload))) + payload
    crc = crc16_ccitt_false(header_and_payload)
    return bytes((SYNC, SYNC)) + header_and_payload + struct.pack("<H", crc)


class ReplyParser:
    """Incrementally extract and validate firmware reply frames."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, int, int, int, int, int]]:
        self.buffer.extend(data)
        replies = []
        while True:
            start = self.buffer.find(bytes((SYNC, SYNC)))
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == bytes((SYNC,)) else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 7:
                break
            payload_length = self.buffer[6]
            frame_length = 9 + payload_length
            if len(self.buffer) < frame_length:
                break
            frame = bytes(self.buffer[:frame_length])
            del self.buffer[:frame_length]
            expected = struct.unpack_from("<H", frame, frame_length - 2)[0]
            if crc16_ccitt_false(frame[2:-2]) != expected:
                continue
            if frame[2] != VERSION or frame[4] != MSG_REPLY or payload_length != 6:
                continue
            status, reason, left, right = struct.unpack_from("<BBhh", frame, 7)
            replies.append((frame[3], frame[5], status, reason, left, right))
        return replies


def send_and_wait(port: serial.Serial, parser: ReplyParser, node: int,
                  sequence: int, left_rpm: int, right_rpm: int) -> tuple[int, int, int, int] | None:
    """Send one node's frame, then wait before using the half-duplex bus again."""
    port.write(make_command(node, sequence, left_rpm, right_rpm))
    deadline = time.monotonic() + REPLY_WAIT_SECONDS
    while time.monotonic() < deadline:
        incoming = port.read(port.in_waiting or 1)
        for reply_node, reply_sequence, status, reason, left, right in parser.feed(incoming):
            if reply_node == node and reply_sequence == sequence:
                return status, reason, left, right
    return None


class KeyReader:
    """Read single keys without waiting for Enter on Windows, macOS, or Linux."""

    def __enter__(self) -> "KeyReader":
        if not sys.stdin.isatty():
            raise RuntimeError("Run this controller in an interactive terminal.")
        if os.name == "nt":
            import msvcrt

            self._msvcrt = msvcrt
        elif os.name == "posix":
            import termios
            import tty

            self._termios = termios
            self._fd = sys.stdin.fileno()
            self._old_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        else:
            raise RuntimeError("Unsupported operating system for keyboard input.")
        return self

    def __exit__(self, *_: object) -> None:
        if os.name == "posix":
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old_settings)

    def read(self) -> str | None:
        if os.name == "nt":
            if self._msvcrt.kbhit():
                key = self._msvcrt.getwch()
                # Ignore the second byte of Windows extended keys (arrows, etc.).
                if key in ("\x00", "\xe0"):
                    self._msvcrt.getwch()
                    return None
                return key.lower()
        else:
            import select

            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                return sys.stdin.read(1).lower()
        return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="USB serial port, e.g. /dev/ttyUSB0 or /dev/cu.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=BAUD, help=f"Serial baud rate (default: {BAUD})")
    parser.add_argument("--rpm", type=int, default=40, help="Wheel RPM for movement (1..135, default: 40)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1 <= args.rpm <= 135:
        raise SystemExit("--rpm must be between 1 and 135 (firmware limit).")

    sequences = {node: 0 for node in TARGET_NODES}
    left, right = 0, 0
    last_sent = 0.0
    node_status = {node: "waiting" for node in TARGET_NODES}
    parser = ReplyParser()
    print("Navibot serial teleop | W forward | S reverse | A pivot left | D pivot right")
    print("SPACE stop | X or Ctrl-C quit. Select a direction; its command repeats at 10 Hz until changed.")
    print("This two-motor firmware has no sideways/strafe command. Ensure the robot is safely supported.")

    try:
        with KeyReader() as keys:
            with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=0.2) as port:
                # Opening a Nano serial port can reset it. Give setup time to finish.
                time.sleep(2.0)
                port.reset_input_buffer()
                try:
                    while True:
                        key = keys.read()
                        changed = False
                        exiting = False
                        if key == "w":
                            left, right = args.rpm, args.rpm
                            changed = True
                        elif key == "s":
                            left, right = -args.rpm, -args.rpm
                            changed = True
                        elif key == "a":
                            left, right = -args.rpm, args.rpm
                            changed = True
                        elif key == "d":
                            left, right = args.rpm, -args.rpm
                            changed = True
                        elif key == " ":
                            left, right = 0, 0
                            changed = True
                        elif key == "x":
                            left, right = 0, 0
                            changed = True
                            exiting = True
                        else:
                            exiting = False

                        now = time.monotonic()
                        if changed or now - last_sent >= HEARTBEAT_SECONDS:
                            for node in TARGET_NODES:
                                reply = send_and_wait(
                                    port, parser, node, sequences[node], left, right
                                )
                                if reply is None:
                                    node_status[node] = "no reply"
                                else:
                                    status, reason, measured_left, measured_right = reply
                                    if status == 0:
                                        node_status[node] = (
                                            f"ACK L:{measured_left} R:{measured_right} RPM"
                                        )
                                    else:
                                        node_status[node] = f"NACK reason:{reason}"
                                sequences[node] = (sequences[node] + 1) & 0xFF
                            last_sent = now

                        sys.stdout.write(
                            f"\rCommand L:{left:>4} R:{right:>4} RPM | "
                            + " | ".join(
                                f"0x{node:02X} {node_status[node]}" for node in TARGET_NODES
                            )
                            + "       "
                        )
                        sys.stdout.flush()
                        if exiting:
                            break
                finally:
                    # Stop the motors before closing the serial port on normal exit or Ctrl-C.
                    for node in TARGET_NODES:
                        send_and_wait(port, parser, node, sequences[node], 0, 0)
                        sequences[node] = (sequences[node] + 1) & 0xFF
                    port.flush()
                    print("\nStop command sent.")
    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
