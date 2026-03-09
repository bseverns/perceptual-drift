"""Process supervision helpers for operator-facing runtime control."""

from __future__ import annotations

import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


class StarterSupervisor:
    """Owns start/stop/status for scripts/starter_up.sh."""

    def __init__(
        self, *, script_path: Path, log_path: Path, cwd: Path
    ) -> None:
        self.script_path = script_path
        self.log_path = log_path
        self.cwd = cwd
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._argv: List[str] = []
        self._started_at: float = 0.0
        self._last_exit_code: Optional[int] = None
        self._last_error: str = ""

    def _refresh_locked(self) -> None:
        if self._proc is None:
            return
        code = self._proc.poll()
        if code is None:
            return
        self._last_exit_code = int(code)
        self._proc = None

    def _build_argv(self, options: Dict[str, Any]) -> List[str]:
        argv: List[str] = []

        tracker_mode = str(options.get("tracker_mode", "synthetic")).strip()
        if tracker_mode not in {"synthetic", "camera"}:
            raise ValueError("tracker_mode must be synthetic or camera")
        argv.extend(["--tracker-mode", tracker_mode])

        video_mode = str(options.get("video", "auto")).strip()
        if video_mode not in {"auto", "on", "off"}:
            raise ValueError("video must be auto, on, or off")
        argv.extend(["--video", video_mode])

        serial = str(options.get("serial", "FAKE")).strip() or "FAKE"
        argv.extend(["--serial", serial])

        tracker_host = str(options.get("tracker_host", "127.0.0.1")).strip()
        if tracker_host:
            argv.extend(["--tracker-host", tracker_host])

        for key, flag in (("osc_port", "--osc-port"), ("hz", "--hz")):
            if key not in options:
                continue
            value = int(options[key])
            argv.extend([flag, str(value)])

        for key, flag in (
            ("video_device", "--video-device"),
            ("video_preset", "--video-preset"),
            ("recipe", "--recipe"),
        ):
            raw = str(options.get(key, "")).strip()
            if raw:
                argv.extend([flag, raw])
        return argv

    def start(
        self, options: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        with self._lock:
            self._refresh_locked()
            if self._proc is not None:
                status = self.status_unlocked()
                status["changed"] = False
                return status

            if not self.script_path.is_file():
                raise FileNotFoundError(
                    f"starter script missing: {self.script_path}"
                )

            opts = dict(options or {})
            argv = self._build_argv(opts)
            cmd = ["bash", str(self.script_path), *argv]
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            log_handle = open(self.log_path, "ab")
            self._last_error = ""
            try:
                self._proc = subprocess.Popen(
                    cmd,
                    cwd=str(self.cwd),
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception as exc:
                self._last_error = str(exc)
                raise
            finally:
                log_handle.close()
            self._argv = argv
            self._started_at = time.time()
            self._last_exit_code = None
            status = self.status_unlocked()
            status["changed"] = True
            return status

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            self._refresh_locked()
            if self._proc is None:
                status = self.status_unlocked()
                status["changed"] = False
                return status

            proc = self._proc
            pid = proc.pid
            try:
                os.killpg(pid, signal.SIGINT)
            except ProcessLookupError:
                pass

            try:
                code = proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(pid, signal.SIGKILL)
                code = proc.wait(timeout=3)

            normalized = int(code)
            if normalized < 0:
                normalized = 0
            self._last_exit_code = normalized
            self._proc = None
            status = self.status_unlocked()
            status["changed"] = True
            return status

    def status(self) -> Dict[str, Any]:
        with self._lock:
            self._refresh_locked()
            return self.status_unlocked()

    def status_unlocked(self) -> Dict[str, Any]:
        running = self._proc is not None and self._proc.poll() is None
        return {
            "running": running,
            "pid": self._proc.pid if running and self._proc is not None else 0,
            "started_at": self._started_at if running else 0.0,
            "args": list(self._argv),
            "log_path": str(self.log_path),
            "last_exit_code": (
                self._last_exit_code if self._last_exit_code is not None else 0
            ),
            "last_error": self._last_error,
        }


DEFAULT_REHEARSAL_PROFILES: List[Dict[str, Any]] = [
    {
        "id": "safe_synthetic",
        "name": "Safe Synthetic (Recommended)",
        "description": "No camera, no FC serial. Best first rehearsal profile.",
        "start_options": {
            "tracker_mode": "synthetic",
            "video": "off",
            "serial": "FAKE",
            "tracker_host": "127.0.0.1",
        },
        "checklist": [
            "Run preflight and clear required failures.",
            "Confirm recipe + consent behavior in UI before performance handoff.",
        ],
    },
    {
        "id": "camera_preview",
        "name": "Camera Preview",
        "description": "Camera tracker + auto video; FC still in dry-run mode.",
        "start_options": {
            "tracker_mode": "camera",
            "video": "auto",
            "serial": "FAKE",
            "tracker_host": "127.0.0.1",
        },
        "checklist": [
            "Verify /dev/video0 and OpenCV availability in preflight.",
            "Confirm framing and tracker stability before opening consent.",
        ],
    },
    {
        "id": "hardware_dry_run",
        "name": "Hardware Dry Run",
        "description": "Camera + video + default serial path for FC integration drills.",
        "start_options": {
            "tracker_mode": "camera",
            "video": "on",
            "serial": "/dev/ttyUSB0",
            "tracker_host": "127.0.0.1",
        },
        "checklist": [
            "Validate FC serial path before pressing Start.",
            "Keep props-off until consent and stop controls are verified.",
        ],
    },
]


def list_rehearsal_profiles() -> List[Dict[str, Any]]:
    return [
        {**profile, "start_options": dict(profile["start_options"])}
        for profile in DEFAULT_REHEARSAL_PROFILES
    ]


def get_rehearsal_profile(profile_id: str) -> Dict[str, Any]:
    normalized = (profile_id or "").strip()
    for profile in DEFAULT_REHEARSAL_PROFILES:
        if profile["id"] == normalized:
            return {**profile, "start_options": dict(profile["start_options"])}
    raise ValueError(f"unknown rehearsal profile: {profile_id}")


class PreflightRunner:
    """Runs starter doctor checks and parses structured status."""

    _LINE_RE = re.compile(r"^\[doctor\]\s+(ok|warn|fail):\s+(.*)$")

    def __init__(self, *, script_path: Path, cwd: Path) -> None:
        self.script_path = script_path
        self.cwd = cwd

    def run(self, *, strict: bool = False) -> Dict[str, Any]:
        if not self.script_path.is_file():
            return {
                "ok": False,
                "code": 127,
                "checked_at": time.time(),
                "checks": [
                    {
                        "level": "fail",
                        "message": f"Preflight script missing: {self.script_path}",
                    }
                ],
                "required_failures": 1,
                "warnings": 0,
                "summary": "missing preflight script",
                "output_tail": [],
            }

        cmd = ["bash", str(self.script_path)]
        if strict:
            cmd.append("--strict")
        proc = subprocess.run(
            cmd,
            cwd=str(self.cwd),
            capture_output=True,
            text=True,
        )
        output = f"{proc.stdout}{proc.stderr}"
        checks: List[Dict[str, str]] = []
        required_failures = 0
        warnings = 0
        summary = ""
        for line in output.splitlines():
            raw = line.strip()
            if not raw:
                continue
            if raw.startswith("[doctor] summary:"):
                summary = raw
                continue
            match = self._LINE_RE.match(raw)
            if not match:
                continue
            level = match.group(1)
            message = match.group(2)
            checks.append({"level": level, "message": message})
            if level == "fail":
                required_failures += 1
            elif level == "warn":
                warnings += 1

        return {
            "ok": proc.returncode == 0,
            "code": int(proc.returncode),
            "checked_at": time.time(),
            "checks": checks,
            "required_failures": required_failures,
            "warnings": warnings,
            "summary": summary,
            "output_tail": output.splitlines()[-20:],
        }
