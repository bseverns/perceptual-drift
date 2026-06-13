"""CLI parsing and runtime loop for the OSC->MSP bridge."""

from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import serial
from pythonosc import dispatcher, osc_server

from bridge_audit import AuditLogger
from bridge_consent import (
    GESTURE_STATE_KEYS,
    GhostGestureBuffer,
    NEUTRAL_AUX_CHANNELS,
    apply_consent_update,
    gesture_snapshot,
    neutral_rc_channels,
    neutralize_gesture_state,
)
from bridge_mapping import (
    Mapper,
    load_recipe,
    load_yaml,
    resolve_bridge_rate,
    resolve_mode_settings,
)
from bridge_midi import MidiListener
from bridge_msp import DryRunSerial, MSP_SET_RAW_RC, clamp, msp_packet
from config_validation import (
    ValidationError,
    validate_mapping_config,
    validate_midi_mapping_config,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAPPING_PATH = (REPO_ROOT / "config/mapping.yaml").resolve()
DEFAULT_MIDI_MAPPING_PATH = (REPO_ROOT / "config/mappings/midi.yaml").resolve()


def build_arg_parser() -> argparse.ArgumentParser:
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
        help=(
            "Path to the OSC/MSP mapping YAML "
            f"(default: {DEFAULT_MAPPING_PATH})."
        ),
    )
    ap.add_argument(
        "--midi_map",
        default=str(DEFAULT_MIDI_MAPPING_PATH),
        help=(
            "Path to the MIDI mapping YAML used to merge CC/note gestures "
            "into the MSP mapper (default: config/mappings/midi.yaml)."
        ),
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
        help=(
            "Bridge mode name declared under bridge.modes "
            "(smooth, triggers_only, ...)"
        ),
    )
    ap.add_argument(
        "--ghost-mode",
        action="store_true",
        help=(
            "Buffer OSC gestures until consent flips high, then replay them "
            "before going live."
        ),
    )
    ap.add_argument(
        "--ghost-buffer-seconds",
        type=float,
        help=("How many seconds of gesture pre-roll to keep while ghosting"),
    )
    ap.add_argument(
        "--stale-after",
        type=float,
        help=(
            "Seconds of OSC silence before we force gestures back to neutral "
            "while consent stays high. 0 or negative disables the watchdog."
        ),
    )
    return ap


def _load_bridge_config(args, audit: AuditLogger):
    cfg_meta = {}
    if args.recipe:
        recipe_path = Path(args.recipe).expanduser().resolve()
        try:
            cfg, cfg_meta = load_recipe(recipe_path)
        except Exception as exc:
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
        except ValidationError as exc:
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
        return cfg, cfg_meta

    config_path = Path(args.config).expanduser()
    if not config_path.is_absolute():
        config_path = (Path.cwd() / config_path).resolve()
    else:
        config_path = config_path.resolve()
    try:
        cfg = load_yaml(config_path)
    except Exception as exc:
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
    except ValidationError as exc:
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
    return cfg, cfg_meta


def _load_midi_config(args, audit: AuditLogger):
    midi_path = Path(args.midi_map).expanduser()
    if not midi_path.is_absolute():
        midi_path = (Path.cwd() / midi_path).resolve()
    else:
        midi_path = midi_path.resolve()
    try:
        midi_cfg = load_yaml(midi_path)
        validate_midi_mapping_config(midi_cfg, midi_path.name)
    except ValidationError as exc:
        audit.write(
            "midi_mapping_validation",
            status="error",
            message="MIDI mapping validation failed",
            details={"errors": exc.errors},
        )
        raise
    except Exception as exc:
        audit.write(
            "midi_mapping_load",
            status="error",
            message=f"Failed to load MIDI mapping config {midi_path}",
            details={"path": str(midi_path.resolve()), "error": str(exc)},
        )
        raise
    audit.write(
        "midi_mapping_load",
        status="info",
        message="Loaded MIDI mapping config",
        details={"path": str(midi_path.resolve())},
    )
    return midi_cfg, midi_path


def main(argv=None):
    ap = build_arg_parser()
    args = ap.parse_args(argv)

    audit = AuditLogger()
    cfg, _cfg_meta = _load_bridge_config(args, audit)
    midi_cfg, midi_path = _load_midi_config(args, audit)

    mode_settings = resolve_mode_settings(cfg, args.mode)
    hz = resolve_bridge_rate(cfg, args.hz)
    bridge_cfg = cfg.get("bridge", {}) or {}
    ghost_default = bridge_cfg.get("ghost_mode", False)
    ghost_mode_enabled = bool(args.ghost_mode or ghost_default)
    ghost_buffer_seconds = args.ghost_buffer_seconds
    if ghost_buffer_seconds is None:
        ghost_buffer_seconds = float(
            bridge_cfg.get("ghost_buffer_seconds", 2.0)
        )
    stale_after = args.stale_after
    if stale_after is None:
        stale_after = float(bridge_cfg.get("stale_after", 0.0) or 0.0)
    if stale_after <= 0:
        stale_after = 0.0
    mapper = Mapper(cfg, mode=mode_settings)
    ghost_buffer = (
        GhostGestureBuffer(ghost_buffer_seconds)
        if ghost_mode_enabled
        else None
    )
    audit.write(
        "osc_bridge_mode",
        status="info",
        message=f"Bridge mode set to {mode_settings.name} at {hz:.1f} Hz",
        details={
            "mode": mode_settings.name,
            "hz": hz,
            "ghost_mode": ghost_mode_enabled,
            "ghost_buffer_seconds": ghost_buffer_seconds,
            "stale_after": stale_after,
        },
    )

    midi_listener: MidiListener | None = None
    try:
        midi_listener = MidiListener(mapper, midi_cfg, audit)
        midi_listener.start()
    except Exception as exc:
        audit.write(
            "midi_bridge_boot",
            status="error",
            message="Failed to start MIDI listener",
            details={"path": str(midi_path), "error": str(exc)},
        )
        raise

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

    def maybe_record_gesture() -> None:
        if ghost_buffer and mapper.state["consent"] != 1:
            ghost_buffer.record(
                time.monotonic(),
                gesture_snapshot(mapper.state, GESTURE_STATE_KEYS),
            )

    def ghost_status() -> str:
        if not ghost_buffer:
            return "ghost:off"
        mode = "replaying" if ghost_buffer.replaying else "buffering"
        return "ghost:{mode} depth={depth} window={window:.1f}s".format(
            mode=mode,
            depth=ghost_buffer.depth(),
            window=ghost_buffer.window_seconds,
        )

    if args.dry_run:
        ser = DryRunSerial(status_cb=ghost_status if ghost_buffer else None)
        print("[dry-run] Serial writes suppressed; MSP frames logged locally.")
        audit.write(
            "osc_bridge_dry_run",
            status="info",
            message="Dry-run mode active; serial link mocked.",
            details={"serial": args.serial, "baud": args.baud},
        )
    else:
        ser = serial.Serial(args.serial, args.baud, timeout=0.01)

    last_osc_time = time.monotonic()
    stale_logged = False

    def _mark_osc_tick() -> None:
        nonlocal last_osc_time, stale_logged
        last_osc_time = time.monotonic()
        if stale_logged:
            audit.write(
                "osc_stale_guard",
                status="info",
                message="OSC stream resumed; watchdog un-neutered the sticks.",
            )
            stale_logged = False

    def on_alt(addr, *vals):
        mapper.state["alt"] = float(clamp(vals[0], -1.0, 1.0))
        _mark_osc_tick()
        maybe_record_gesture()

    def on_lat(addr, *vals):
        mapper.state["lat"] = float(clamp(vals[0], -1.0, 1.0))
        _mark_osc_tick()
        maybe_record_gesture()

    def on_yaw(addr, *vals):
        mapper.state["yaw"] = float(clamp(vals[0], -1.0, 1.0))
        _mark_osc_tick()
        maybe_record_gesture()

    def on_crowd(addr, *vals):
        mapper.state["crowd"] = float(clamp(vals[0], 0.0, 1.0))
        _mark_osc_tick()
        maybe_record_gesture()

    def on_consent(addr, *vals):
        prev = mapper.state["consent"]
        result = apply_consent_update(
            mapper.state, vals[0], ghost_buffer=ghost_buffer
        )
        _mark_osc_tick()
        if result.replay_started:
            audit.write(
                "ghost_buffer_replay",
                status="info",
                message=(
                    "Consent opened; replaying buffered gestures "
                    "before live passthrough."
                ),
                details={
                    "buffer_depth": result.replay_depth,
                    "window_seconds": ghost_buffer.window_seconds,
                },
            )
            if args.dry_run:
                print(
                    (
                        "[dry-run][ghost] Replaying {count} buffered "
                        "frames before live."
                    ).format(count=result.replay_depth)
                )
        if result.changed:
            status = "armed" if result.current == 1 else "disarmed"
            audit.write(
                "consent_toggle",
                status=status,
                details={"value": result.current, "osc_addr": addr},
            )
        if result.dropped_to_idle and prev == 1:
            audit.write(
                "osc_bridge_stream",
                status="neutralized",
                message="Consent dropped; neutral frame primed.",
            )

    disp = dispatcher.Dispatcher()
    disp.map(cfg["osc"]["address_space"]["altitude"], on_alt)
    disp.map(cfg["osc"]["address_space"]["lateral"], on_lat)
    disp.map(cfg["osc"]["address_space"]["yaw"], on_yaw)
    disp.map(cfg["osc"]["address_space"]["crowd"], on_crowd)
    disp.map(cfg["osc"]["address_space"]["consent"], on_consent)

    server = osc_server.ThreadingOSCUDPServer(("0.0.0.0", args.osc_port), disp)
    print(f"OSC listening on {server.server_address}")
    audit.write(
        "osc_bridge_boot",
        status="info",
        message=f"OSC→MSP bridge listening on {server.server_address}",
    )

    last = 0
    update_interval = 1.0 / hz
    neutral_rc = neutral_rc_channels()

    streaming_live = False

    try:
        while True:
            now = time.monotonic()
            is_stale = False
            if stale_after and now - last_osc_time > stale_after:
                is_stale = True
                if not stale_logged:
                    audit.write(
                        "osc_stale_guard",
                        status="warning",
                        message=(
                            "OSC stream stalled past {:.2f}s; soft-neutralizing "
                            "live gestures."
                        ).format(stale_after),
                    )
                    stale_logged = True
            if ghost_buffer:
                snapshot = ghost_buffer.snapshot_for(now)
                if snapshot:
                    mapper.state.update(snapshot)
            if now - last > update_interval:
                if is_stale and mapper.state["consent"] == 1:
                    neutralize_gesture_state(mapper.state)
                rc_channels, aux_channels = mapper.apply()
                payload = struct.pack(
                    "<8H",
                    rc_channels[0],
                    rc_channels[1],
                    rc_channels[2],
                    rc_channels[3],
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
                        NEUTRAL_AUX_CHANNELS[0],
                        NEUTRAL_AUX_CHANNELS[1],
                        NEUTRAL_AUX_CHANNELS[2],
                        NEUTRAL_AUX_CHANNELS[3],
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
        if midi_listener:
            midi_listener.stop()
        ser.close()
        audit.write(
            "osc_bridge_shutdown",
            status="closed",
            message="Serial link closed{suffix}.".format(
                suffix=" (dry-run stub)" if args.dry_run else ""
            ),
        )
    return 0
