#!/usr/bin/env python3
"""OSC-to-MSP control bridge for hijacking Betaflight with wild data.

This script is intentionally over-commented so future operators can wire up
their own inputs without spelunking the MSP spec from scratch.  Read the notes,
experiment shamelessly, and keep the props pointed away from your face.
"""

import argparse
import math
import struct
import time

import serial
import yaml
from pythonosc import dispatcher, osc_server

# --- MSP minimal helpers (subset) ---
# The header and command IDs live here so you can tweak them without grepping.
MSP_HEADER = b"\x24\x4D\x3C"  # '$M<' — Betaflight's Minimal Serial Protocol
MSP_SET_RAW_RC = 200  # command ID for pushing raw RC channel values


def msp_packet(cmd, payload=b""):
    """Wrap a payload in MSP framing with the expected XOR checksum."""

    size = len(payload)
    checksum = (size ^ cmd ^ sum(payload)) & 0xFF
    return MSP_HEADER + bytes([size, cmd]) + payload + bytes([checksum])


def clamp(value, lower, upper):
    """Clamp ``value`` into ``[lower, upper]`` with no surprises."""

    return max(lower, min(upper, value))


def map_float_to_rc(value, gain=1.0, center=1500, span=400):
    """Map normalized [-1, 1] values into Betaflight-friendly RC microseconds."""

    return int(clamp(center + value * span * gain, 1100, 1900))


class Mapper:
    def __init__(self, cfg):
        # Stash the YAML config so we can grab mapping knobs everywhere.
        self.cfg = cfg
        # ``state`` holds the most recent values coming in from OSC.  Every
        # handler updates these keys, and ``apply`` transforms them into RC µs.
        self.state = {
            "alt": 0.0,  # normalized altitude vibe: -1 = drop, +1 = rise
            "lat": 0.0,  # lateral strafe: -1 = left, +1 = right
            "yaw": 0.0,  # yaw twist: -1 = counter-clockwise, +1 = clockwise
            "crowd": 0.0,  # extra dimension for creative routing or lights
            "consent": 0,  # kill-switch: 1 streams MSP packets, 0 parks them
        }

    def expo(self, x, k=0.5):
        """Push the midpoint flatter while keeping the sign intact."""

        return math.copysign((abs(x) ** (1 + k)), x)

    def apply(self):
        """Convert the current sensor state into four RC channel values."""

        alt = self.state["alt"]
        lat = self.state["lat"]
        yaw = self.state["yaw"]

        # Optionally run the altitude through an exponential curve.  Expo keeps
        # hover tweaks chill while still letting you punch the throttle when
        # the performance needs drama.
        if self.cfg["mapping"]["altitude"]["curve"] == "expo":
            alt = self.expo(alt, 0.3)
        lat_gain = self.cfg["mapping"]["lateral"]["gain"]
        yaw_bias = self.cfg["mapping"]["yaw_bias"]["bias"]

        rc_roll = map_float_to_rc(lat, gain=lat_gain)
        # Pitch tips forward proportional to lateral magnitude so sideways
        # slides keep their swagger.  Yank or reshape it if that's not your jam.
        rc_pitch = map_float_to_rc(-abs(lat) * 0.2)
        # Throttle squishes [-1, 1] into [0, 1] before mapping into microseconds.
        rc_thr = map_float_to_rc(
            (alt + 1) / 2 - 0.5,
            gain=self.cfg["mapping"]["altitude"]["gain"],
        )
        # Yaw bias lets you trim out mechanical drift without touching hardware.
        rc_yaw = map_float_to_rc(yaw + yaw_bias)

        return [rc_roll, rc_pitch, rc_thr, rc_yaw]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--serial', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--config', default='../../config/mapping.yaml')
    ap.add_argument('--osc_port', type=int, default=9000)
    args = ap.parse_args()

    # Slurp the mapping configuration.  This is where you wire OSC paths to
    # channel names and tweak curves/gains without touching code.
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    mapper = Mapper(cfg)

    # Fire up the serial link to the flight controller.  MSP is just bytes over
    # UART, so as long as the port is right you're golden.
    ser = serial.Serial(args.serial, args.baud, timeout=0.01)

    def on_alt(addr, *vals):
        """OSC handler for altitude channel."""

        mapper.state["alt"] = float(clamp(vals[0], -1.0, 1.0))

    def on_lat(addr, *vals):
        """OSC handler for lateral (roll) channel."""

        mapper.state["lat"] = float(clamp(vals[0], -1.0, 1.0))

    def on_yaw(addr, *vals):
        """OSC handler for yaw channel."""

        mapper.state["yaw"] = float(clamp(vals[0], -1.0, 1.0))

    def on_crowd(addr, *vals):
        """OSC handler for crowd/aux data."""

        mapper.state["crowd"] = float(clamp(vals[0], 0.0, 1.0))

    def on_consent(addr, *vals):
        """OSC handler for the go/no-go toggle."""

        mapper.state["consent"] = int(vals[0])

    disp = dispatcher.Dispatcher()
    disp.map(cfg["osc"]["address_space"]["altitude"], on_alt)
    disp.map(cfg["osc"]["address_space"]["lateral"], on_lat)
    disp.map(cfg["osc"]["address_space"]["yaw"], on_yaw)
    disp.map(cfg["osc"]["address_space"]["crowd"], on_crowd)
    disp.map(cfg["osc"]["address_space"]["consent"], on_consent)

    # Spin up an OSC server that listens for the incoming sensor party.
    server = osc_server.ThreadingOSCUDPServer(("0.0.0.0", args.osc_port), disp)
    print(f"OSC listening on {server.server_address}")

    # ``last`` timestamps the previous MSP push so we can throttle updates.
    last = 0
    try:
        while True:
            now = time.time()
            if now - last > 0.02:  # 50 Hz RC updates
                rc = mapper.apply()
                # 8 channels: roll, pitch, throttle, yaw, AUX1..AUX4.  If you
                # need more, change the struct format and append extra values.
                payload = struct.pack(
                    '<8H',
                    rc[0],
                    rc[1],
                    rc[2],
                    rc[3],
                    1500,
                    1500,
                    1500,
                    1500,
                )
                pkt = msp_packet(MSP_SET_RAW_RC, payload)
                if mapper.state["consent"] == 1:
                    ser.write(pkt)
                else:
                    # Leaving consent low keeps the drone armed but chilled so
                    # you can rehearse without sending MSP spam downstream.
                    pass
                last = now
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        # Close the serial port so the next run doesn't start with a fight.
        ser.close()

if __name__ == "__main__":
    main()
