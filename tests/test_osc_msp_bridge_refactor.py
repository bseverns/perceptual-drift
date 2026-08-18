import threading
import time
from types import SimpleNamespace

import mido
import pytest
from pythonosc import dispatcher, udp_client
from pythonosc.osc_server import ThreadingOSCUDPServer

import scripts.check_stack as check_stack


bridge_runtime = check_stack.bridge._bridge_runtime


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
    assert mapper.state["lat"] == 1.0
    assert 0.49 < mapper.state["crowd"] < 0.51

    listener._handle_message(
        mido.Message("note_on", channel=1, note=60, velocity=100)
    )
    assert mapper.state["consent"] == 1

    listener._handle_message(
        mido.Message("note_on", channel=1, note=60, velocity=100)
    )
    listener._handle_message(
        mido.Message("control_change", channel=0, control=10, value=0)
    )

    assert mapper.state["lat"] == 0.0
    assert mapper.state["crowd"] == 0.0
    assert mapper.state["consent"] == 0


def test_midi_consent_drop_neutralizes_gestures():
    bridge = check_stack.bridge
    mapper = SimpleNamespace(
        state={
            "alt": 0.8,
            "lat": -0.7,
            "yaw": 0.6,
            "crowd": 0.9,
            "consent": 1,
        }
    )
    listener = bridge.MidiListener(
        mapper,
        {
            "gestures": [
                {
                    "type": "note",
                    "note_number": 60,
                    "target": "consent",
                    "mode": "gate",
                }
            ]
        },
        audit=SimpleNamespace(write=lambda *args, **kwargs: None),
    )

    listener._handle_message(mido.Message("note_off", note=60, velocity=0))

    assert mapper.state == {
        "alt": 0.0,
        "lat": 0.0,
        "yaw": 0.0,
        "crowd": 0.0,
        "consent": 0,
    }


def test_start_osc_server_serves_in_background_thread():
    received = threading.Event()
    disp = dispatcher.Dispatcher()
    disp.map("/pd/test", lambda _addr, *_vals: received.set())
    server = ThreadingOSCUDPServer(("127.0.0.1", 0), disp)
    thread = bridge_runtime._start_osc_server(server)
    host, port = server.server_address

    try:
        client = udp_client.SimpleUDPClient(host, port)
        client.send_message("/pd/test", 1)
        assert received.wait(timeout=2)
        assert thread.is_alive()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_main_binds_recipe_osc_port_when_cli_port_is_default(monkeypatch):
    seen = {}
    cfg = {
        "osc": {
            "port": 9123,
            "address_space": {
                "altitude": "/alt",
                "lateral": "/lat",
                "yaw": "/yaw",
                "crowd": "/crowd",
                "consent": "/consent",
            },
        },
        "mapping": {"altitude": {}, "lateral": {}},
    }

    class FakeServer:
        def __init__(self, address, _disp):
            seen["address"] = address
            self.server_address = address

    class FakeMidiListener:
        def __init__(self, *_args, **_kwargs):
            pass

        def start(self):
            pass

    monkeypatch.setattr(
        bridge_runtime, "_load_bridge_config", lambda _args, _audit: (cfg, {})
    )
    monkeypatch.setattr(
        bridge_runtime,
        "_load_midi_config",
        lambda _args, _audit: ({}, "midi.yaml"),
    )
    monkeypatch.setattr(bridge_runtime, "MidiListener", FakeMidiListener)
    monkeypatch.setattr(
        bridge_runtime.osc_server, "ThreadingOSCUDPServer", FakeServer
    )
    monkeypatch.setattr(
        bridge_runtime,
        "_start_osc_server",
        lambda _server: (_ for _ in ()).throw(RuntimeError("stop after bind")),
    )

    try:
        bridge_runtime.main(["--dry-run", "--recipe", "recipe.yaml"])
    except RuntimeError as exc:
        assert str(exc) == "stop after bind"

    assert seen["address"] == ("127.0.0.1", 9123)


def test_bridge_requires_explicit_bind_for_lan_exposure(monkeypatch, capsys):
    seen = {}
    cfg = {
        "osc": {
            "port": 9000,
            "address_space": {
                "altitude": "/alt",
                "lateral": "/lat",
                "yaw": "/yaw",
                "crowd": "/crowd",
                "consent": "/consent",
            },
        },
        "mapping": {"altitude": {}, "lateral": {}},
    }

    class FakeServer:
        def __init__(self, address, _disp):
            seen["address"] = address
            self.server_address = address

    class FakeMidiListener:
        def __init__(self, *_args, **_kwargs):
            pass

        def start(self):
            pass

    monkeypatch.setattr(
        bridge_runtime, "_load_bridge_config", lambda _args, _audit: (cfg, {})
    )
    monkeypatch.setattr(
        bridge_runtime,
        "_load_midi_config",
        lambda _args, _audit: ({}, "midi.yaml"),
    )
    monkeypatch.setattr(bridge_runtime, "MidiListener", FakeMidiListener)
    monkeypatch.setattr(
        bridge_runtime.osc_server, "ThreadingOSCUDPServer", FakeServer
    )
    monkeypatch.setattr(
        bridge_runtime,
        "_start_osc_server",
        lambda _server: (_ for _ in ()).throw(RuntimeError("stop after bind")),
    )

    with pytest.raises(RuntimeError, match="stop after bind"):
        bridge_runtime.main(["--dry-run", "--bind", "0.0.0.0"])

    assert seen["address"] == ("0.0.0.0", 9000)
    assert "Any host that can reach this socket" in capsys.readouterr().err
