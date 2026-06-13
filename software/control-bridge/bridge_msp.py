"""MSP packet helpers and serial dry-run utilities."""

from __future__ import annotations

import struct
import time
from typing import Callable

MSP_HEADER = b"\x24\x4d\x3c"
MSP_SET_RAW_RC = 200


def msp_packet(cmd, payload=b""):
    """Wrap a payload in MSP framing with the expected XOR checksum."""

    size = len(payload)
    checksum = (size ^ cmd ^ sum(payload)) & 0xFF
    return MSP_HEADER + bytes([size, cmd]) + payload + bytes([checksum])


def clamp(value, lower, upper):
    """Clamp ``value`` into ``[lower, upper]`` with no surprises."""

    return max(lower, min(upper, value))


def map_float_to_rc(value, gain=1.0, center=1500, span=400):
    """Map [-1, 1] floats into Betaflight-friendly RC microseconds."""

    return int(clamp(center + value * span * gain, 1100, 1900))


class DryRunSerial:
    """Lightweight stand-in for ``serial.Serial`` during dry runs."""

    def __init__(self, status_cb: Callable[[], str] | None = None):
        self.byte_count = 0
        self._last_report = time.time()
        self._last_frame = None
        self._status_cb = status_cb

    def write(self, payload: bytes) -> None:
        self.byte_count += len(payload)
        if len(payload) >= 7 and payload.startswith(MSP_HEADER):
            size = payload[3]
            cmd = payload[4]
            if cmd == MSP_SET_RAW_RC and size == 16:
                frame = struct.unpack("<8H", payload[5 : 5 + size])
                self._last_frame = frame
        now = time.time()
        if now - self._last_report >= 1.0:
            suffix = ""
            if self._status_cb:
                suffix = f" | {self._status_cb()}"
                if self._last_frame:
                    rc = list(self._last_frame[:4])
                    aux = list(self._last_frame[4:])
                    message = (
                        "[dry-run] RC={} AUX={} (frame bytes={} total={}){}"
                    )
                    print(
                        message.format(
                            rc,
                            aux,
                            len(payload),
                            self.byte_count,
                            suffix,
                        )
                    )
            else:
                total = self.byte_count
                message = "[dry-run] would stream {} bytes ({} total so far)"
                print(message.format(len(payload), total) + suffix)
            self._last_report = now

    def close(self) -> None:
        if self._last_frame:
            rc = list(self._last_frame[:4])
            aux = list(self._last_frame[4:])
            print(f"[dry-run] last frame RC={rc} AUX={aux}")
        print(f"[dry-run] serial stub wrote {self.byte_count} bytes total")
