import time
from types import SimpleNamespace

import mido

import scripts.check_stack as check_stack


def test_consent_update_replays_buffer_and_neutralizes_on_drop():
    bridge = check_stack.bridge
    state = {
        "alt": 0.4,
        "lat": -0.3,
        "yaw": 0.2,
        "crowd": 0.8,
        "consent": 0,
    }
    ghost = bridge.GhostGestureBuffer(window_seconds=2.0)
    now = time.monotonic()
    ghost.record(now - 0.5, {"alt": 0.1, "lat": 0.2, "yaw": 0.3, "crowd": 0.4})
    ghost.record(now - 0.1, {"alt": 0.5, "lat": 0.6, "yaw": 0.7, "crowd": 0.8})

    armed = bridge.apply_consent_update(state, 1, ghost_buffer=ghost)

    assert armed.changed is True
    assert armed.current == 1
    assert armed.replay_started is True
    assert armed.replay_depth == 2

    dropped = bridge.apply_consent_update(state, 0, ghost_buffer=ghost)

    assert dropped.changed is True
    assert dropped.dropped_to_idle is True
    assert state["consent"] == 0
    assert state["alt"] == 0.0
    assert state["lat"] == 0.0
    assert state["yaw"] == 0.0
    assert state["crowd"] == 0.0
    assert ghost.depth() == 0


def test_midi_listener_maps_cc_and_toggle_notes_without_runtime_thread():
    bridge = check_stack.bridge
    mapper = SimpleNamespace(
        state={
            "alt": 0.0,
            "lat": 0.0,
            "yaw": 0.0,
            "crowd": 0.0,
            "consent": 0,
        }
    )
    midi_cfg = {
        "input": {"channel": 2},
        "gestures": [
            {"type": "cc", "cc_number": 10, "target": "lateral"},
            {
                "type": "cc",
                "cc_number": 11,
                "target": "crowd",
                "response": "unipolar",
            },
            {
                "type": "note",
                "note_number": 60,
                "target": "consent",
                "mode": "toggle",
            },
        ],
    }
    listener = bridge.MidiListener(
        mapper,
        midi_cfg,
        audit=SimpleNamespace(write=lambda *args, **kwargs: None),
    )

    listener._handle_message(
        mido.Message("control_change", channel=1, control=10, value=127)
    )
    listener._handle_message(
        mido.Message("control_change", channel=1, control=11, value=64)
    )
    listener._handle_message(
        mido.Message("note_on", channel=1, note=60, velocity=100)
    )
    listener._handle_message(
        mido.Message("note_on", channel=1, note=60, velocity=100)
    )
    listener._handle_message(
        mido.Message("control_change", channel=0, control=10, value=0)
    )

    assert mapper.state["lat"] == 1.0
    assert 0.49 < mapper.state["crowd"] < 0.51
    assert mapper.state["consent"] == 0
