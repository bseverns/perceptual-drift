from pathlib import Path
from unittest.mock import patch

import pytest

from software.operator_ui.supervisor import StarterSupervisor


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
    with patch("software.operator_ui.supervisor.subprocess.Popen", return_value=fake) as popen:
        status = sup.start({"tracker_mode": "synthetic", "video": "off", "osc_port": 9000})
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
    with patch("software.operator_ui.supervisor.subprocess.Popen", return_value=fake):
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
