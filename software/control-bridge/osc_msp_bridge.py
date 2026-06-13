#!/usr/bin/env python3
"""OSC-to-MSP control bridge for hijacking Betaflight with wild data.

This module now serves as the compatibility surface for the bridge. Runtime
logic lives in focused sibling modules, but the public names stay re-exported
here so existing imports and scripts keep working unchanged.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

BRIDGE_DIR = Path(__file__).resolve().parent
if str(BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(BRIDGE_DIR))

_bridge_audit = importlib.import_module("bridge_audit")
_bridge_consent = importlib.import_module("bridge_consent")
_bridge_mapping = importlib.import_module("bridge_mapping")
_bridge_midi = importlib.import_module("bridge_midi")
_bridge_msp = importlib.import_module("bridge_msp")
_bridge_runtime = importlib.import_module("bridge_runtime")
_config_validation = importlib.import_module("config_validation")

AuditLogger = _bridge_audit.AuditLogger
GhostGestureBuffer = _bridge_consent.GhostGestureBuffer
apply_consent_update = _bridge_consent.apply_consent_update
gesture_snapshot = _bridge_consent.gesture_snapshot
neutral_rc_channels = _bridge_consent.neutral_rc_channels
neutralize_gesture_state = _bridge_consent.neutralize_gesture_state
to_binary_state = _bridge_consent.to_binary_state
Mapper = _bridge_mapping.Mapper
deep_merge = _bridge_mapping.deep_merge
load_recipe = _bridge_mapping.load_recipe
load_yaml = _bridge_mapping.load_yaml
resolve_bridge_rate = _bridge_mapping.resolve_bridge_rate
resolve_mode_settings = _bridge_mapping.resolve_mode_settings
MIDI_BACKEND = _bridge_midi.MIDI_BACKEND
MIDI_BACKEND_ERROR = _bridge_midi.MIDI_BACKEND_ERROR
MidiListener = _bridge_midi.MidiListener
normalize_cc = _bridge_midi.normalize_cc
DryRunSerial = _bridge_msp.DryRunSerial
MSP_HEADER = _bridge_msp.MSP_HEADER
MSP_SET_RAW_RC = _bridge_msp.MSP_SET_RAW_RC
clamp = _bridge_msp.clamp
map_float_to_rc = _bridge_msp.map_float_to_rc
msp_packet = _bridge_msp.msp_packet
main = _bridge_runtime.main
ModeSettings = _config_validation.ModeSettings
ValidationError = _config_validation.ValidationError
validate_mapping_config = _config_validation.validate_mapping_config
validate_midi_mapping_config = _config_validation.validate_midi_mapping_config

__all__ = [
    "AuditLogger",
    "DryRunSerial",
    "GhostGestureBuffer",
    "MIDI_BACKEND",
    "MIDI_BACKEND_ERROR",
    "MSP_HEADER",
    "MSP_SET_RAW_RC",
    "Mapper",
    "MidiListener",
    "ModeSettings",
    "ValidationError",
    "apply_consent_update",
    "clamp",
    "deep_merge",
    "gesture_snapshot",
    "load_recipe",
    "load_yaml",
    "main",
    "map_float_to_rc",
    "msp_packet",
    "neutral_rc_channels",
    "neutralize_gesture_state",
    "normalize_cc",
    "resolve_bridge_rate",
    "resolve_mode_settings",
    "to_binary_state",
    "validate_mapping_config",
    "validate_midi_mapping_config",
]


if __name__ == "__main__":
    raise SystemExit(main())
