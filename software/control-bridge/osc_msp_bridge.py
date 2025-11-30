#!/usr/bin/env python3
"""OSC-to-MSP control bridge for hijacking Betaflight with wild data.

This script is intentionally over-commented so future operators can
wire their own inputs without spelunking the MSP spec from scratch.
Read the notes, experiment shamelessly, and keep the props pointed away
from your face.

References worth opening in a browser tab while you read this file:

* MSP command table —
  https://github.com/betaflight/betaflight.com/
  blob/master/docs/development/API/MSP-Extensions.md
* python-osc docs — https://pypi.org/project/python-osc/
* Betaflight raw RC ranges —
  https://github.com/betaflight/betaflight/wiki/RC-Setup
"""

import argparse
import json
import math
import os
import random
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
import yaml
from pythonosc import dispatcher, osc_server

BRIDGE_DIR = Path(__file__).resolve().parent
if str(BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(BRIDGE_DIR))

from config_validation import ModeSettings, ValidationError, validate_mapping_config

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAPPING_PATH = (REPO_ROOT / "config/mapping.yaml").resolve()

# --- MSP minimal helpers (subset) ---
# The header and command IDs live here so you can tweak them without grepping.
# The MSP header is literally the bytes ``$M<`` — you see them in serial
# logs when sniffing a radio link. Betaflight expects exactly this framing.
MSP_HEADER = b"\x24\x4d\x3c"  # '$M<' — Betaflight's Minimal Serial Protocol
# ID 200 = SET_RAW_RC.  Newer Betaflight builds still accept this despite
# other MSP v2 command expansions.  If you want to play with v2, start with
# the link above and pack CRCs instead of the XOR checksum used here.
MSP_SET_RAW_RC = 200  # command ID for pushing raw RC channel values


def deep_merge(base, override):
    """Recursively merge ``override`` into ``base`` returning a new dict."""

    if not isinstance(base, dict) or not isinstance(override, dict):
        return override
    merged = dict(base)
    for key, value in override.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_yaml(path):
    """Read YAML/JSON from ``path`` and return the parsed structure."""

    with open(path) as fh:
        return yaml.safe_load(fh)


def load_recipe(recipe_path):
    """Load a recipe file and return (control_cfg, metadata dict)."""

    recipe_path = Path(recipe_path).expanduser().resolve()
    data = load_yaml(recipe_path)
    if not isinstance(data, dict):
        raise ValueError(f"Recipe at {recipe_path} must parse into a mapping")

    control_section = (
        data.get("control_bridge")
        or data.get("control")
        or data.get("bridge")
        or {k: v for k, v in data.items() if k in {"mapping", "osc", "bridge", "extends"}}
    )
    if not control_section:
        raise ValueError(
            "Recipe {path} needs a 'control_bridge' section describing "
            "the MSP mapping".format(path=recipe_path)
        )

    base_cfg = {}
    extends = control_section.get("extends")
    if extends:
        base_path = (recipe_path.parent / extends).resolve()
        base_cfg = load_yaml(base_path) or {}

    overlay = {k: v for k, v in control_section.items() if k != "extends"}
    cfg = deep_merge(base_cfg, overlay)

    metadata = {
        "name": data.get("name", recipe_path.stem),
        "slug": data.get("slug"),
        "description": data.get("description", ""),
        "intent": data.get("intent", ""),
        "leds": data.get("leds", {}),
        "video_pipeline": data.get("video_pipeline", {}),
    }
    return cfg, metadata


def msp_packet(cmd, payload=b""):
    """Wrap a payload in MSP framing with the expected XOR checksum.

    MSP v1 packets are: ``'$M<' + [payload_size] + [cmd_id] + payload +
    checksum``.
    The checksum is the XOR of size, command, and each payload byte. See the
    Betaflight wiki for the canonical pseudocode.  This helper sticks to the
    tiny subset we need: write raw RC channel values.
    """

    size = len(payload)
    checksum = (size ^ cmd ^ sum(payload)) & 0xFF
    return MSP_HEADER + bytes([size, cmd]) + payload + bytes([checksum])


def clamp(value, lower, upper):
    """Clamp ``value`` into ``[lower, upper]`` with no surprises."""

    return max(lower, min(upper, value))


def map_float_to_rc(value, gain=1.0, center=1500, span=400):
    """Map [-1, 1] floats into Betaflight-friendly RC microseconds.

    Betaflight arms easiest when channels live between 1100 ↔ 1900 µs and sit
    around 1500 µs at rest.  The ``span`` of 400 gives us ±400 µs around the
    center; tweak if you want more throw.
    """

    return int(clamp(center + value * span * gain, 1100, 1900))


def resolve_mode_settings(cfg, requested_mode=None):
    """Return :class:`ModeSettings` for the requested or default bridge mode."""

    bridge_cfg = cfg.get("bridge", {}) or {}
    modes_cfg = bridge_cfg.get("modes") or {}
    mode_name = requested_mode or bridge_cfg.get("mode", "smooth")
    if not modes_cfg:
        return ModeSettings(
            name=mode_name,
            description="",
            deadzone_boost={},
            gain_scale={},
            jitter_scale=1.0,
            neutral_rc=False,
            aux_strategy="full",
        )
    if mode_name not in modes_cfg:
        raise ValidationError([f"bridge.mode '{mode_name}' not found in bridge.modes"])
    mode_cfg = modes_cfg[mode_name] or {}
    return ModeSettings(
        name=mode_name,
        description=mode_cfg.get("description", ""),
        deadzone_boost=mode_cfg.get("deadzone_boost", {}),
        gain_scale=mode_cfg.get("gain_scale", {}),
        jitter_scale=float(mode_cfg.get("jitter_scale", 1.0)),
        neutral_rc=bool(mode_cfg.get("neutral_rc", False)),
        aux_strategy=str(mode_cfg.get("aux_strategy", "full")),
    )


def resolve_bridge_rate(cfg, requested_hz=None):
    bridge_cfg = cfg.get("bridge", {}) or {}
    hz = requested_hz or bridge_cfg.get("hz", 50)
    try:
        hz = float(hz)
    except (TypeError, ValueError) as exc:  # noqa: BLE001
        raise ValidationError(["bridge.hz must be numeric"])
    if hz <= 0:
        raise ValidationError(["bridge.hz must be > 0"])
    return hz


class AuditLogger:
    def __init__(self):
        repo_root = Path(__file__).resolve().parents[2]
        log_dir_env = os.environ.get("PERCEPTUAL_DRIFT_LOG_DIR")
        if log_dir_env:
            candidate = Path(log_dir_env)
            if candidate.is_absolute():
                log_dir = candidate
            else:
                log_dir = repo_root / candidate
        else:
            log_dir = repo_root / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / "ops_events.jsonl"
        self.operator = (
            os.environ.get("OPERATOR_ID")
            or os.environ.get("USER")
            or os.environ.get("USERNAME")
            or "unknown"
        )
        self.host = os.environ.get("HOSTNAME", "unknown_host")

    def write(self, action, status="info", message=None, details=None):
        event = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "operator": self.operator,
            "host": self.host,
            "action": action,
            "status": status,
        }
        if message:
            event["message"] = message
        if details is not None:
            event["details"] = details
        with self.log_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(event, ensure_ascii=False) + "\n")


class DryRunSerial:
    """Lightweight stand-in for ``serial.Serial`` during dry runs."""

    def __init__(self):
        self.byte_count = 0
        self._last_report = time.time()
        self._last_frame = None

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
            if self._last_frame:
                rc = list(self._last_frame[:4])
                aux = list(self._last_frame[4:])
                print(
                    "[dry-run] RC={} AUX={} (frame bytes={} total={})".format(
                        rc,
                        aux,
                        len(payload),
                        self.byte_count,
                    )
                )
            else:
                total = self.byte_count
                print(
                    (
                        "[dry-run] would stream {} bytes (total {} bytes so " "far)"
                    ).format(
                        len(payload),
                        total,
                    )
                )
            self._last_report = now

    def close(self) -> None:
        if self._last_frame:
            rc = list(self._last_frame[:4])
            aux = list(self._last_frame[4:])
            print(f"[dry-run] last frame RC={rc} AUX={aux}")
        print(f"[dry-run] serial stub wrote {self.byte_count} bytes total")


class Mapper:
    def __init__(self, cfg, *, mode: ModeSettings | None = None):
        # Stash the YAML config so we can grab mapping knobs everywhere.  The
        # schema is documented in README.md but boils down to ``mapping`` and
        # ``osc.address_space`` dictionaries.
        self.cfg = cfg
        self.mode = mode or resolve_mode_settings(cfg)
        # ``state`` holds the most recent values coming in from OSC.  Every
        # handler updates these keys, and ``apply`` transforms them into RC µs.
        self.state = {
            "alt": 0.0,  # normalized altitude vibe: -1 = drop, +1 = rise
            "lat": 0.0,  # lateral strafe: -1 = left, +1 = right
            "yaw": 0.0,  # yaw twist: -1 = counter-clockwise, +1 = clockwise
            "crowd": 0.0,  # extra dimension for creative routing or lights
            # consent = 0 still streams MSP, but forces a chilled neutral frame
            # so Betaflight keeps seeing a heartbeat while the props stay napping
            "consent": 0,
        }

    def set_mode(self, mode: ModeSettings) -> None:
        self.mode = mode

    def expo(self, x, k=0.5):
        """Push the midpoint flatter while keeping the sign intact.

        Classic radio controllers let you apply exponential curves so the
        center feels gentle while the edges remain aggressive.  Same vibe here.
        """

        return math.copysign((abs(x) ** (1 + k)), x)

    def apply_deadzone(self, value, deadzone):
        """Snap tiny inputs to zero while preserving full-throw range."""

        if deadzone <= 0:
            return value
        if deadzone >= 1:
            return 0.0
        if abs(value) <= deadzone:
            return 0.0
        scaled = (abs(value) - deadzone) / (1 - deadzone)
        return math.copysign(scaled, value)

    def shape_axis(self, axis_name, value):
        """Apply deadzone + curve shaping from the mapping config."""

        axis_cfg = self.cfg["mapping"].get(axis_name, {})
        deadzone = axis_cfg.get("deadzone", 0.0)
        deadzone += self.mode.deadzone_for(axis_name)
        deadzone = clamp(deadzone, 0.0, 0.99)
        shaped = self.apply_deadzone(value, deadzone)
        curve = axis_cfg.get("curve", "linear")
        if curve == "expo":
            strength = axis_cfg.get("expo_strength", axis_cfg.get("expo", 0.5))
            shaped = self.expo(shaped, strength)
        return clamp(shaped, -1.0, 1.0)

    def apply(self):
        """Convert the current sensor state into RC + AUX channel values.

        Returns a tuple ``([roll, pitch, throttle, yaw], [aux1..aux4])`` in RC
        microseconds.
        """

        mapping_cfg = self.cfg.get("mapping", {})

        alt = self.shape_axis("altitude", self.state["alt"])
        lat = self.shape_axis("lateral", self.state["lat"])

        yaw_cfg = mapping_cfg.get("yaw_bias", {})
        yaw_bias = yaw_cfg.get("bias", 0.0)
        jitter = yaw_cfg.get("jitter", 0.0) * self.mode.jitter_scale
        if jitter:
            yaw_bias += random.uniform(-jitter, jitter)
        yaw = clamp(self.state["yaw"] + yaw_bias, -1.0, 1.0)

        lat_gain = mapping_cfg.get("lateral", {}).get("gain", 1.0)
        lat_gain *= self.mode.gain_for("lateral", 1.0)
        alt_gain = mapping_cfg.get("altitude", {}).get("gain", 1.0)
        alt_gain *= self.mode.gain_for("altitude", 1.0)

        rc_roll = map_float_to_rc(lat, gain=lat_gain)
        # Pitch tips forward proportional to lateral magnitude so sideways
        # slides keep their swagger.  Yank or reshape it if that's not your
        # jam.
        rc_pitch = map_float_to_rc(-abs(lat) * 0.2)
        # Throttle squishes [-1, 1] into [-0.5, 0.5] before mapping into
        # microseconds.
        rc_thr = map_float_to_rc(alt / 2, gain=alt_gain)
        # Yaw bias lets you trim out mechanical drift without touching
        # hardware.  ``jitter`` (if non-zero) adds intentional wobble to keep
        # the quad from sticking on a deadband.
        rc_yaw = map_float_to_rc(yaw)

        crowd = clamp(self.state["crowd"], 0.0, 1.0)
        aux_crowd = map_float_to_rc(crowd * 2 - 1)

        glitch_cfg = mapping_cfg.get("glitch_intensity", {})
        glitch_base = glitch_cfg.get("base", 0.0)
        glitch_max = glitch_cfg.get("max", 1.0)
        glitch_norm = clamp(
            glitch_base + (glitch_max - glitch_base) * crowd,
            0.0,
            1.0,
        )
        aux_glitch = map_float_to_rc(glitch_norm * 2 - 1)

        palette = mapping_cfg.get("leds", {}).get("palette", [])
        if palette:
            if len(palette) == 1:
                led_norm = 0.0
            else:
                slot = int(round(crowd * (len(palette) - 1)))
                slot = clamp(slot, 0, len(palette) - 1)
                led_norm = slot / (len(palette) - 1)
        else:
            led_norm = 0.0
        aux_led = map_float_to_rc(led_norm * 2 - 1)

        aux_channels = [aux_crowd, aux_glitch, aux_led, 1500]
        rc_channels = [rc_roll, rc_pitch, rc_thr, rc_yaw]

        if self.mode.neutral_rc:
            rc_channels = [
                map_float_to_rc(0.0),
                map_float_to_rc(0.0),
                map_float_to_rc(-1.0),
                map_float_to_rc(0.0),
            ]
        if self.mode.aux_strategy == "crowd_only":
            aux_channels = [aux_crowd, 1500, 1500, 1500]

        return rc_channels, aux_channels


def main():
    ap = argparse.ArgumentParser(
        description=(
            "Listen for OSC control data and spit Minimal Serial Protocol "
            "packets at a Betaflight flight controller."
        )
    )
    ap.add_argument("--serial", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument(
        "--config",
        default=str(DEFAULT_MAPPING_PATH),
        help=f"Path to the OSC/MSP mapping YAML (default: {DEFAULT_MAPPING_PATH}).",
    )
    ap.add_argument(
        "--recipe",
        help=(
            "Path to a YAML/JSON recipe bundling MSP curves, LED palettes, "
            "and video cues. Overrides --config when provided."
        ),
    )
    ap.add_argument("--osc_port", type=int, default=9000)
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="Skip serial writes and log MSP traffic.",
    )
    ap.add_argument("--hz", type=float, help="Limit MSP frame rate (Hz)")
    ap.add_argument(
        "--mode",
        help="Bridge mode name declared under bridge.modes (smooth, triggers_only, ...)",
    )
    args = ap.parse_args()

    audit = AuditLogger()

    # Slurp the mapping configuration.  This is where you wire OSC paths to
    # channel names and tweak deadzones/curves/gains without touching code.
    # ``mapping.altitude`` and ``mapping.lateral`` now understand ``deadzone``,
    # ``curve`` ("linear" or "expo"), and an optional ``expo_strength`` knob.
    # ``mapping.yaw_bias`` adds deterministic trim plus optional ``jitter`` to
    # keep mechanical deadbands honest.  AUX payloads borrow the OSC ``crowd``
    # stream plus ``mapping.leds`` and ``mapping.glitch_intensity`` values
    # downstream. Recipes tack on narrative intent so operators know the vibe.
    cfg_meta = {}
    if args.recipe:
        recipe_path = Path(args.recipe).expanduser().resolve()
        try:
            cfg, cfg_meta = load_recipe(recipe_path)
        except Exception as exc:  # noqa: BLE001
            audit.write(
                "recipe_load",
                status="error",
                message=f"Failed to load recipe from {recipe_path}",
                details={
                    "path": str(recipe_path.resolve()),
                    "error": str(exc),
                },
            )
            raise
        try:
            validate_mapping_config(cfg, recipe_path.name)
        except ValidationError as exc:  # noqa: BLE001
            audit.write(
                "recipe_error",
                status="error",
                message=f"Recipe validation failed for {recipe_path.name}",
                details={"errors": exc.errors},
            )
            raise
        if cfg_meta:
            print(
                "Loaded recipe '{name}' — {intent}".format(
                    name=cfg_meta.get("name", recipe_path.stem),
                    intent=cfg_meta.get("intent", ""),
                ).strip()
            )
        audit.write(
            "recipe_load",
            status="info",
            message="Loaded recipe '{name}'".format(
                name=cfg_meta.get("name", recipe_path.stem)
            ),
            details={
                "path": str(recipe_path.resolve()),
                "intent": cfg_meta.get("intent"),
                "slug": cfg_meta.get("slug"),
                "osc_port": cfg.get("osc", {}).get("port"),
            },
        )
    else:
        config_path = Path(args.config).expanduser()
        if not config_path.is_absolute():
            config_path = (Path.cwd() / config_path).resolve()
        else:
            config_path = config_path.resolve()
        try:
            cfg = load_yaml(config_path)
        except Exception as exc:  # noqa: BLE001
            audit.write(
                "mapping_load",
                status="error",
                message=f"Failed to load mapping config {config_path}",
                details={
                    "path": str(config_path.resolve()),
                    "error": str(exc),
                },
            )
            raise
        try:
            validate_mapping_config(cfg, config_path.name)
        except ValidationError as exc:  # noqa: BLE001
            audit.write(
                "mapping_validation",
                status="error",
                message="Mapping validation failed",
                details={"errors": exc.errors},
            )
            raise
        audit.write(
            "mapping_load",
            status="info",
            message="Loaded mapping config",
            details={"path": str(config_path.resolve())},
        )
    mode_settings = resolve_mode_settings(cfg, args.mode)
    hz = resolve_bridge_rate(cfg, args.hz)
    mapper = Mapper(cfg, mode=mode_settings)
    audit.write(
        "osc_bridge_mode",
        status="info",
        message=f"Bridge mode set to {mode_settings.name} at {hz:.1f} Hz",
        details={"mode": mode_settings.name, "hz": hz},
    )

    default_port = ap.get_default("osc_port")
    osc_port = args.osc_port
    if args.recipe and osc_port == default_port:
        recipe_port = cfg.get("osc", {}).get("port")
        if recipe_port:
            audit.write(
                "osc_port_override",
                status="info",
                message=f"Recipe requested OSC port {recipe_port}",
                details={
                    "source": "recipe",
                    "path": str(Path(args.recipe).resolve()),
                },
            )
        osc_port = recipe_port or default_port
    elif osc_port != default_port:
        audit.write(
            "osc_port_override",
            status="info",
            message=f"Operator requested OSC port {osc_port}",
            details={"source": "cli"},
        )

    # Fire up the serial link to the flight controller.  MSP is just bytes over
    # UART, so as long as the port is right you're golden.
    if args.dry_run:
        ser = DryRunSerial()
        print("[dry-run] Serial writes suppressed; MSP frames logged locally.")
        audit.write(
            "osc_bridge_dry_run",
            status="info",
            message="Dry-run mode active; serial link mocked.",
            details={"serial": args.serial, "baud": args.baud},
        )
    else:
        ser = serial.Serial(args.serial, args.baud, timeout=0.01)

    def on_alt(addr, *vals):
        """OSC handler for altitude channel.

        ``vals`` is a tuple because python-osc lets senders include multiple
        values.  We only care about the first element and clamp it so wild OSC
        rigs can't arm the drone with bogus numbers.
        """

        mapper.state["alt"] = float(clamp(vals[0], -1.0, 1.0))

    def on_lat(addr, *vals):
        """OSC handler for lateral (roll) channel.

        Map left/right crowd shifts (or whatever sensor you wire in) to roll.
        """

        mapper.state["lat"] = float(clamp(vals[0], -1.0, 1.0))

    def on_yaw(addr, *vals):
        """OSC handler for yaw channel.

        Yaw bias in the config lets you trim mechanical drift without touching
        Betaflight Configurator.
        """

        mapper.state["yaw"] = float(clamp(vals[0], -1.0, 1.0))

    def on_crowd(addr, *vals):
        """OSC handler for crowd/aux data.

        We pipe this into AUX channels for LED/glitch consumers.  Keep it
        normalized 0..1 to make re-use simple.
        """

        mapper.state["crowd"] = float(clamp(vals[0], 0.0, 1.0))

    def on_consent(addr, *vals):
        """OSC handler for the go/no-go toggle.

        Consent-off does **not** halt MSP writes.  Instead we keep the bytes
        flowing with a neutral frame: sticks parked at center, throttle locked
        low, yaw recentered, AUX channels mid-stick.  Betaflight keeps seeing a
        heartbeat while the quad stays sedated — rehearsal mode without yanking
        the USB cable.  Treat it as an extra arming layer stacked atop the radio
        failsafes.
        """

        prev = mapper.state["consent"]
        new_val = int(vals[0])
        mapper.state["consent"] = 1 if new_val == 1 else 0
        if mapper.state["consent"] != prev:
            status = "armed" if mapper.state["consent"] == 1 else "disarmed"
            audit.write(
                "consent_toggle",
                status=status,
                details={"value": mapper.state["consent"], "osc_addr": addr},
            )
        if mapper.state["consent"] != 1 and prev == 1:
            mapper.state.update(
                {
                    "alt": 0.0,
                    "lat": 0.0,
                    "yaw": 0.0,
                    "crowd": 0.0,
                }
            )
            audit.write(
                "osc_bridge_stream",
                status="neutralized",
                message="Consent dropped; neutral frame primed.",
            )

    disp = dispatcher.Dispatcher()
    # Map each OSC address path in the YAML into its handler.  The defaults
    # follow ``/pd/*`` because the Processing sketch ships those paths.
    disp.map(cfg["osc"]["address_space"]["altitude"], on_alt)
    disp.map(cfg["osc"]["address_space"]["lateral"], on_lat)
    disp.map(cfg["osc"]["address_space"]["yaw"], on_yaw)
    disp.map(cfg["osc"]["address_space"]["crowd"], on_crowd)
    disp.map(cfg["osc"]["address_space"]["consent"], on_consent)

    # Spin up an OSC server that listens for the incoming sensor party.
    server = osc_server.ThreadingOSCUDPServer(
        ("0.0.0.0", args.osc_port),
        disp,
    )
    print(f"OSC listening on {server.server_address}")
    audit.write(
        "osc_bridge_boot",
        status="info",
        message=f"OSC→MSP bridge listening on {server.server_address}",
    )

    # ``last`` timestamps the previous MSP push so we can throttle updates.
    last = 0
    update_interval = 1.0 / hz
    # Pre-pack the "everyone take a breath" frame so we can blast it whenever
    # consent drops.  Roll/pitch/yaw park at 1500 µs, throttle clamps low, and
    # AUX channels chill at mid-stick.
    neutral_rc = (
        map_float_to_rc(0.0),
        map_float_to_rc(0.0),
        map_float_to_rc(-1.0),
        map_float_to_rc(0.0),
    )
    neutral_aux = (1500, 1500, 1500, 1500)

    streaming_live = False

    try:
        while True:
            now = time.time()
            if now - last > update_interval:
                rc_channels, aux_channels = mapper.apply()
                # 8 channels: roll, pitch, throttle, yaw, AUX1..AUX4.  If you
                # need more, change the struct format and append extra values.
                payload = struct.pack(
                    "<8H",
                    # Roll, pitch, throttle, yaw — exactly what Betaflight
                    # expects for channels 1–4.  These ints are already µs.
                    rc_channels[0],
                    rc_channels[1],
                    rc_channels[2],
                    rc_channels[3],
                    # AUX1..AUX4 now carry the crowd/glitch/LED payloads so
                    # downstream systems can actually consume the YAML knobs.
                    aux_channels[0],
                    aux_channels[1],
                    aux_channels[2],
                    aux_channels[3],
                )
                pkt = msp_packet(MSP_SET_RAW_RC, payload)
                if mapper.state["consent"] == 1:
                    ser.write(pkt)
                    if not streaming_live:
                        audit.write(
                            "osc_bridge_stream",
                            status="armed",
                            message="Live MSP stream resumed.",
                            details={"mode": mapper.mode.name, "hz": hz},
                        )
                        streaming_live = True
                else:
                    chill_payload = struct.pack(
                        "<8H",
                        neutral_rc[0],
                        neutral_rc[1],
                        neutral_rc[2],
                        neutral_rc[3],
                        neutral_aux[0],
                        neutral_aux[1],
                        neutral_aux[2],
                        neutral_aux[3],
                    )
                    ser.write(msp_packet(MSP_SET_RAW_RC, chill_payload))
                    if streaming_live:
                        audit.write(
                            "osc_bridge_stream",
                            status="neutralized",
                            message="Consent gating forced neutral MSP frame.",
                            details={"mode": mapper.mode.name, "hz": hz},
                        )
                        streaming_live = False
                last = now
            time.sleep(0.005)
    except KeyboardInterrupt:
        audit.write(
            "osc_bridge_shutdown",
            status="info",
            message="Operator interrupted bridge (Ctrl+C).",
        )
    finally:
        # Close the serial port so the next run doesn't start with a fight.
        ser.close()
        audit.write(
            "osc_bridge_shutdown",
            status="closed",
            message="Serial link closed{suffix}.".format(
                suffix=" (dry-run stub)" if args.dry_run else ""
            ),
        )


if __name__ == "__main__":
    main()
