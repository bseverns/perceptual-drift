from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from software.operator_ui.supervisor import (
    PreflightRunner,
    StarterSupervisor,
    get_rehearsal_profile,
    list_rehearsal_profiles,
)


class FakeProc:
    def __init__(self, pid: int = 1234):
        self.pid = pid
        self._running = True
        self.returncode = None

    def poll(self):
        return None if self._running else self.returncode

    def wait(self, timeout=None):
        self._running = False
        self.returncode = 0
        return 0


def make_supervisor(tmp_path: Path) -> StarterSupervisor:
    script = tmp_path / "starter_up.sh"
    script.write_text("#!/usr/bin/env bash\nexit 0\n")
    return StarterSupervisor(
        script_path=script,
        log_path=tmp_path / "supervisor.log",
        cwd=tmp_path,
    )


def test_supervisor_start_and_status(tmp_path):
    sup = make_supervisor(tmp_path)
    fake = FakeProc(pid=4321)
    with patch(
        "software.operator_ui.supervisor.subprocess.Popen", return_value=fake
    ) as popen:
        status = sup.start(
            {"tracker_mode": "synthetic", "video": "off", "osc_port": 9000}
        )
    assert status["running"] is True
    assert status["pid"] == 4321
    assert status["changed"] is True
    cmd = popen.call_args.args[0]
    assert "--tracker-mode" in cmd
    assert "--video" in cmd
    assert "--osc-port" in cmd


def test_supervisor_stop_transitions_to_not_running(tmp_path):
    sup = make_supervisor(tmp_path)
    fake = FakeProc(pid=7777)
    with patch(
        "software.operator_ui.supervisor.subprocess.Popen", return_value=fake
    ):
        sup.start()
    with patch("software.operator_ui.supervisor.os.killpg") as killpg:
        stopped = sup.stop()
    killpg.assert_called_once()
    assert stopped["running"] is False
    assert stopped["last_exit_code"] == 0
    assert stopped["changed"] is True


def test_supervisor_rejects_invalid_mode(tmp_path):
    sup = make_supervisor(tmp_path)
    with pytest.raises(ValueError):
        sup.start({"tracker_mode": "invalid"})


def test_rehearsal_profile_catalog_and_lookup():
    profiles = list_rehearsal_profiles()
    ids = {p["id"] for p in profiles}
    assert "safe_synthetic" in ids
    assert get_rehearsal_profile("camera_preview")["id"] == "camera_preview"
    with pytest.raises(ValueError):
        get_rehearsal_profile("unknown_profile")


def test_preflight_runner_parses_doctor_output(tmp_path):
    script = tmp_path / "starter_doctor.sh"
    script.write_text("#!/usr/bin/env bash\nexit 0\n")
    runner = PreflightRunner(script_path=script, cwd=tmp_path)
    fake = SimpleNamespace(
        returncode=1,
        stdout=(
            "[doctor] ok: Python runtime (python3)\n"
            "[doctor] warn: No /dev/video0 found; video preview may not launch.\n"
            "[doctor] fail: Config validation failed.\n"
            "[doctor] summary: required_failures=1 warnings=1\n"
        ),
        stderr="",
    )
    with patch(
        "software.operator_ui.supervisor.subprocess.run", return_value=fake
    ):
        report = runner.run()
    assert report["ok"] is False
    assert report["required_failures"] == 1
    assert report["warnings"] == 1
    assert len(report["checks"]) == 3
