from argparse import Namespace
from pathlib import Path

from software.operator_ui import rehearsal


class FakeProc:
    def __init__(self, pid: int = 2468):
        self.pid = pid


def make_args(tmp_path: Path, command: str = "start") -> Namespace:
    return Namespace(
        command=command,
        host="127.0.0.1",
        port=8088,
        runtime_dir=str(tmp_path / "runtime" / "rehearsal"),
        api_token="",
        wait_timeout=1.0,
    )


def test_safe_rehearsal_start_boots_ui_and_runtime(
    tmp_path, monkeypatch, capsys
):
    args = make_args(tmp_path)
    calls = []

    def fake_api_request(base_url, path, **kwargs):
        calls.append((base_url, path, kwargs))
        if path == "/api/runtime/start":
            return {"ok": True, "supervisor": {"running": True, "pid": 9001}}
        if path == "/api/consent":
            return {"ok": True, "state": {"consent_state": 0}}
        return {"ok": True}

    monkeypatch.setattr(
        rehearsal.subprocess, "Popen", lambda *a, **k: FakeProc()
    )
    monkeypatch.setattr(rehearsal, "_wait_for_health", lambda *a, **k: None)
    monkeypatch.setattr(rehearsal, "_api_request", fake_api_request)
    monkeypatch.setattr(rehearsal, "_stop_runtime_fallback", lambda: None)

    assert rehearsal._start(args) == 0

    output = capsys.readouterr().out
    runtime_dir = Path(args.runtime_dir)
    meta = rehearsal._load_metadata(runtime_dir / "safe_rehearsal.json")

    assert "[safe-rehearsal] operator UI: http://127.0.0.1:8088" in output
    assert "Confirm consent stays OFF" in output
    assert (runtime_dir / "operator_ui.pid").read_text().strip() == "2468"
    assert (runtime_dir / "operator_ui.token").read_text().strip()
    assert meta["base_url"] == "http://127.0.0.1:8088"
    assert calls[0][1] == "/api/runtime/start"
    assert calls[0][2]["payload"] == rehearsal.DEFAULT_START_OPTIONS
    assert calls[1][1] == "/api/consent"
    assert calls[1][2]["payload"] == {"consent": 0}


def test_safe_rehearsal_status_reports_running_state(
    tmp_path, monkeypatch, capsys
):
    args = make_args(tmp_path, command="status")
    runtime_dir = Path(args.runtime_dir)
    runtime_dir.mkdir(parents=True)
    (runtime_dir / "operator_ui.pid").write_text("2468\n")
    (runtime_dir / "operator_ui.token").write_text("test-token\n")
    rehearsal._write_metadata(
        runtime_dir / "safe_rehearsal.json",
        {"host": "127.0.0.1", "port": 8088},
    )

    def fake_api_request(base_url, path, **kwargs):
        if path == "/api/runtime/supervisor":
            return {"ok": True, "supervisor": {"running": True, "pid": 9001}}
        if path == "/api/state":
            return {"ok": True, "state": {"consent_state": 0}}
        raise AssertionError(path)

    monkeypatch.setattr(
        rehearsal, "_pid_running", lambda pid: pid in {2468, 9001}
    )
    monkeypatch.setattr(rehearsal, "_api_request", fake_api_request)

    assert rehearsal._status(args) == 0

    output = capsys.readouterr().out
    assert "operator UI: running (pid 2468)" in output
    assert "starter runtime: running (pid 9001)" in output
    assert "consent: 0" in output


def test_safe_rehearsal_stop_uses_runtime_api_and_cleans_pid_file(
    tmp_path, monkeypatch, capsys
):
    args = make_args(tmp_path, command="stop")
    runtime_dir = Path(args.runtime_dir)
    runtime_dir.mkdir(parents=True)
    pid_file = runtime_dir / "operator_ui.pid"
    pid_file.write_text("2468\n")
    (runtime_dir / "operator_ui.token").write_text("test-token\n")
    rehearsal._write_metadata(
        runtime_dir / "safe_rehearsal.json",
        {"host": "127.0.0.1", "port": 8088},
    )
    calls = []

    monkeypatch.setattr(rehearsal, "_pid_running", lambda pid: pid == 2468)
    monkeypatch.setattr(
        rehearsal,
        "_api_request",
        lambda base_url, path, **kwargs: calls.append((base_url, path, kwargs))
        or {"ok": True},
    )
    monkeypatch.setattr(rehearsal, "_signal_process", lambda pid, sig: None)
    monkeypatch.setattr(rehearsal, "_wait_for_exit", lambda pid, timeout: True)
    monkeypatch.setattr(rehearsal, "_stop_runtime_fallback", lambda: None)

    assert rehearsal._stop(args) == 0

    output = capsys.readouterr().out
    assert calls[0][1] == "/api/runtime/stop"
    assert not pid_file.exists()
    assert "[safe-rehearsal] stopped" in output
