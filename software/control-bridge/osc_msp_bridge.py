#!/usr/bin/env python3
import argparse, sys, time, struct, json, yaml, math
import serial
from pythonosc import dispatcher, osc_server

# --- MSP minimal helpers (subset) ---
MSP_HEADER = b'\x24\x4D\x3C'  # '$M<'
MSP_SET_RAW_RC = 200

def msp_packet(cmd, payload=b''):
    size = len(payload)
    checksum = (size ^ cmd ^ sum(payload)) & 0xFF
    return MSP_HEADER + bytes([size, cmd]) + payload + bytes([checksum])

def clamp(v, lo, hi): return max(lo, min(hi, v))

def map_float_to_rc(v, gain=1.0, center=1500, span=400):
    # expects v in [-1,1] -> RC range [1100,1900]
    return int(clamp(center + v * span * gain, 1100, 1900))

class Mapper:
    def __init__(self, cfg):
        self.cfg = cfg
        self.state = {"alt":0.0,"lat":0.0,"yaw":0.0,"crowd":0.0,"consent":0}

    def expo(self, x, k=0.5):
        return math.copysign((abs(x)**(1+k)), x)

    def apply(self):
        alt = self.state["alt"]
        lat = self.state["lat"]
        yaw = self.state["yaw"]
        crowd = self.state["crowd"]

        # Curves
        if self.cfg["mapping"]["altitude"]["curve"] == "expo":
            alt = self.expo(alt, 0.3)
        lat_gain = self.cfg["mapping"]["lateral"]["gain"]
        yaw_bias = self.cfg["mapping"]["yaw_bias"]["bias"]

        rc_roll  = map_float_to_rc(lat,  gain=lat_gain)
        rc_pitch = map_float_to_rc(-abs(lat)*0.2)  # slight forward nudge
        rc_thr   = map_float_to_rc((alt+1)/2 - 0.5, gain=self.cfg["mapping"]["altitude"]["gain"])
        rc_yaw   = map_float_to_rc(yaw + yaw_bias)

        return [rc_roll, rc_pitch, rc_thr, rc_yaw]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--serial', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--config', default='../../config/mapping.yaml')
    ap.add_argument('--osc_port', type=int, default=9000)
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    mapper = Mapper(cfg)

    ser = serial.Serial(args.serial, args.baud, timeout=0.01)

    def on_alt(addr, *vals):
        mapper.state["alt"] = float(clamp(vals[0], -1.0, 1.0))
    def on_lat(addr, *vals):
        mapper.state["lat"] = float(clamp(vals[0], -1.0, 1.0))
    def on_yaw(addr, *vals):
        mapper.state["yaw"] = float(clamp(vals[0], -1.0, 1.0))
    def on_crowd(addr, *vals):
        mapper.state["crowd"] = float(clamp(vals[0], 0.0, 1.0))
    def on_consent(addr, *vals):
        mapper.state["consent"] = int(vals[0])

    disp = dispatcher.Dispatcher()
    disp.map(cfg["osc"]["address_space"]["altitude"], on_alt)
    disp.map(cfg["osc"]["address_space"]["lateral"], on_lat)
    disp.map(cfg["osc"]["address_space"]["yaw"], on_yaw)
    disp.map(cfg["osc"]["address_space"]["crowd"], on_crowd)
    disp.map(cfg["osc"]["address_space"]["consent"], on_consent)

    server = osc_server.ThreadingOSCUDPServer(("0.0.0.0", args.osc_port), disp)
    print(f"OSC listening on {server.server_address}")

    last = 0
    try:
        while True:
            now = time.time()
            if now - last > 0.02:  # 50 Hz RC updates
                rc = mapper.apply()
                # 8 channels: roll, pitch, throttle, yaw, AUX1..AUX4
                payload = struct.pack('<8H', rc[0], rc[1], rc[2], rc[3], 1500,1500,1500,1500)
                pkt = msp_packet(MSP_SET_RAW_RC, payload)
                if mapper.state["consent"] == 1:
                    ser.write(pkt)
                last = now
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()
