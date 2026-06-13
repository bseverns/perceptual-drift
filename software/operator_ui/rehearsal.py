#!/usr/bin/env python3
"""Single-command safe rehearsal launcher for no-hardware runs."""

from __future__ import annotations

import argparse
import json
import os
import secrets
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict
from urllib import error, request

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUNTIME_DIR = REPO_ROOT / "runtime" / "rehearsal"
DEFAULT_START_OPTIONS: Dict[str, Any] = {
    "tracker_mode": "synthetic",
    "video": "off",
    "serial": "FAKE",
    "tracker_host": "127.0.0.1",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Start or control a safe no-hardware rehearsal."
    )
    parser.add_argument(
        "command",
        nargs="?",
        choices=("start", "status", "stop"),
        default="start",
        help="Lifecycle command (default: start).",
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="Operator UI bind host."
    )
    parser.add_argument(
        "--port", type=int, default=8088, help="Operator UI bind port."
    )
    parser.add_argument(
        "--runtime-dir",
        default=str(DEFAULT_RUNTIME_DIR),
        help="Directory for rehearsal pid/log/token files.",
    )
    parser.add_argument(
        "--api-token",
        default=os.environ.get("OPERATOR_API_TOKEN", ""),
        help="Optional fixed token for the operator UI API.",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the operator UI to accept requests.",
    )
    return parser.parse_args()


def _runtime_paths(runtime_dir: Path) -> Dict[str, Path]:
    return {
        "meta": runtime_dir / "safe_rehearsal.json",
        "pid": runtime_dir / "operator_ui.pid",
        "log": runtime_dir / "operator_ui.log",
        "token": runtime_dir / "operator_ui.token",
        "starter_log": runtime_dir / "starter_supervisor.log",
        "session_exports": runtime_dir / "sessions",
    }


def _display_host(host: str) -> str:
    return "127.0.0.1" if host in {"0.0.0.0", "::"} else host


def _base_url(host: str, port: int) -> str:
    return f"http://{_display_host(host)}:{int(port)}"


def _read_pid(path: Path) -> int:
    try:
        return int(path.read_text().strip())
    except (OSError, ValueError):
        return 0


def _pid_running(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _signal_process(pid: int, sig: signal.Signals) -> None:
    if pid <= 0:
        return
    try:
        os.killpg(pid, sig)
        return
    except ProcessLookupError:
        return
    except OSError:
        pass
    try:
        os.kill(pid, sig)
    except OSError:
        return


def _wait_for_exit(pid: int, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not _pid_running(pid):
            return True
        time.sleep(0.1)
    return not _pid_running(pid)


def _api_request(
    base_url: str,
    path: str,
    *,
    token: str = "",
    method: str = "GET",
    payload: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    headers: Dict[str, str] = {}
    data = None
    if token:
        headers["Authorization"] = f"Bearer {token}"
    if payload is not None:
        headers["Content-Type"] = "application/json"
        data = json.dumps(payload).encode("utf-8")
    req = request.Request(
        f"{base_url}{path}",
        headers=headers,
        data=data,
        method=method,
    )
    try:
        with request.urlopen(req, timeout=2) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except error.HTTPError as exc:
        body = exc.read().decode("utf-8")
        try:
            parsed = json.loads(body)
        except json.JSONDecodeError:
            parsed = {"ok": False, "error": body or str(exc)}
        raise RuntimeError(parsed.get("error", f"HTTP {exc.code}")) from exc


def _wait_for_health(base_url: str, timeout: float) -> None:
    deadline = time.time() + timeout
    last_error = "operator UI did not respond"
    while time.time() < deadline:
        try:
            payload = _api_request(base_url, "/api/health")
        except (
            Exception
        ) as exc:  # pragma: no cover - exercised via timeout path
            last_error = str(exc)
            time.sleep(0.2)
            continue
        if payload.get("ok") is True:
            return
        last_error = str(payload)
        time.sleep(0.2)
    raise RuntimeError(last_error)


def _load_metadata(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return {}


def _write_metadata(path: Path, payload: Dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n")


def _stop_runtime_fallback() -> None:
    runtime_dir = REPO_ROOT / "runtime" / "starter_bundle"
    for name in ("bridge.pid", "tracker.pid", "video.pid"):
        pid_file = runtime_dir / name
        pid = _read_pid(pid_file)
        if not _pid_running(pid):
            pid_file.unlink(missing_ok=True)
            continue
        _signal_process(pid, signal.SIGINT)
        if not _wait_for_exit(pid, 2):
            _signal_process(pid, signal.SIGKILL)
            _wait_for_exit(pid, 1)
        pid_file.unlink(missing_ok=True)


def _read_token(paths: Dict[str, Path], arg_token: str) -> str:
    token = arg_token.strip()
    if token:
        return token
    try:
        return paths["token"].read_text().strip()
    except OSError:
        return ""


def _collect_status(args: argparse.Namespace) -> Dict[str, Any]:
    runtime_dir = Path(args.runtime_dir).resolve()
    paths = _runtime_paths(runtime_dir)
    meta = _load_metadata(paths["meta"])
    host = str(meta.get("host", args.host))
    port = int(meta.get("port", args.port))
    base_url = _base_url(host, port)
    token = _read_token(paths, args.api_token)
    pid = _read_pid(paths["pid"])
    running = _pid_running(pid)
    status: Dict[str, Any] = {
        "operator_ui_running": running,
        "operator_ui_pid": pid if running else 0,
        "base_url": base_url,
        "token_path": str(paths["token"]),
        "operator_ui_log": str(paths["log"]),
        "starter_log": str(paths["starter_log"]),
        "runtime_dir": str(runtime_dir),
        "runtime_running": False,
        "runtime_pid": 0,
        "consent_state": None,
    }
    if not running:
        return status
    try:
        supervisor = _api_request(
            base_url, "/api/runtime/supervisor", token=token
        )
        state = _api_request(base_url, "/api/state", token=token)
    except Exception:
        return status
    sup = supervisor.get("supervisor", {})
    runtime_state = state.get("state", {})
    status["runtime_running"] = bool(sup.get("running", False))
    status["runtime_pid"] = int(sup.get("pid", 0) or 0)
    status["consent_state"] = runtime_state.get("consent_state")
    return status


def _start(args: argparse.Namespace) -> int:
    runtime_dir = Path(args.runtime_dir).resolve()
    runtime_dir.mkdir(parents=True, exist_ok=True)
    paths = _runtime_paths(runtime_dir)
    pid = _read_pid(paths["pid"])
    if _pid_running(pid):
        status = _collect_status(args)
        print(
            "[safe-rehearsal] already running "
            f"(operator-ui pid {status['operator_ui_pid']})"
        )
        print(f"[safe-rehearsal] operator UI: {status['base_url']}")
        print("[safe-rehearsal] stop with: pd-safe-rehearsal stop")
        return 0

    token = _read_token(paths, args.api_token)
    if not token:
        token = secrets.token_urlsafe(16)
    paths["token"].write_text(token + "\n")
    base_url = _base_url(args.host, args.port)
    cmd = [
        sys.executable,
        "-m",
        "software.operator_ui.server",
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--api-token",
        token,
        "--starter-log",
        str(paths["starter_log"]),
        "--session-export-dir",
        str(paths["session_exports"]),
    ]

    log_handle = open(paths["log"], "ab")
    proc = None
    try:
        proc = subprocess.Popen(
            cmd,
            cwd=str(REPO_ROOT),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    finally:
        log_handle.close()

    paths["pid"].write_text(f"{proc.pid}\n")

    try:
        _wait_for_health(base_url, args.wait_timeout)
        _api_request(
            base_url,
            "/api/runtime/start",
            token=token,
            method="POST",
            payload=DEFAULT_START_OPTIONS,
        )
        _api_request(
            base_url,
            "/api/consent",
            token=token,
            method="POST",
            payload={"consent": 0},
        )
    except Exception as exc:
        _signal_process(proc.pid, signal.SIGINT)
        if not _wait_for_exit(proc.pid, 3):
            _signal_process(proc.pid, signal.SIGKILL)
            _wait_for_exit(proc.pid, 1)
        paths["pid"].unlink(missing_ok=True)
        _stop_runtime_fallback()
        raise RuntimeError(f"failed to start rehearsal: {exc}") from exc

    _write_metadata(
        paths["meta"],
        {
            "host": args.host,
            "port": args.port,
            "base_url": base_url,
            "operator_ui_pid_file": str(paths["pid"]),
            "operator_ui_log": str(paths["log"]),
            "operator_ui_token_file": str(paths["token"]),
            "starter_log": str(paths["starter_log"]),
            "starter_runtime_dir": str(
                REPO_ROOT / "runtime" / "starter_bundle"
            ),
            "started_at": time.time(),
        },
    )
    print(f"[safe-rehearsal] operator UI: {base_url}")
    print(f"[safe-rehearsal] API token: {token}")
    print(f"[safe-rehearsal] logs: {paths['log']}")
    print(
        "[safe-rehearsal] starter logs: "
        f"{REPO_ROOT / 'runtime' / 'starter_bundle'}"
    )
    print("[safe-rehearsal] next:")
    print("  1. Open the operator UI URL in a browser.")
    print("  2. Enter the API token if the UI prompts for it.")
    print("  3. Confirm consent stays OFF before changing recipes.")
    print("  4. Stop with: pd-safe-rehearsal stop")
    return 0


def _status(args: argparse.Namespace) -> int:
    status = _collect_status(args)
    ui_state = (
        f"running (pid {status['operator_ui_pid']})"
        if status["operator_ui_running"]
        else "stopped"
    )
    runtime_state = (
        f"running (pid {status['runtime_pid']})"
        if status["runtime_running"]
        else "stopped"
    )
    print(f"[safe-rehearsal] operator UI: {ui_state}")
    print(f"[safe-rehearsal] starter runtime: {runtime_state}")
    print(f"[safe-rehearsal] url: {status['base_url']}")
    if status["consent_state"] is not None:
        print(f"[safe-rehearsal] consent: {status['consent_state']}")
    print(f"[safe-rehearsal] token file: {status['token_path']}")
    print(f"[safe-rehearsal] log: {status['operator_ui_log']}")
    return 0


def _stop(args: argparse.Namespace) -> int:
    runtime_dir = Path(args.runtime_dir).resolve()
    paths = _runtime_paths(runtime_dir)
    meta = _load_metadata(paths["meta"])
    host = str(meta.get("host", args.host))
    port = int(meta.get("port", args.port))
    base_url = _base_url(host, port)
    token = _read_token(paths, args.api_token)
    pid = _read_pid(paths["pid"])

    if _pid_running(pid):
        try:
            _api_request(
                base_url,
                "/api/runtime/stop",
                token=token,
                method="POST",
                payload={},
            )
        except Exception:
            pass

        _signal_process(pid, signal.SIGINT)
        if not _wait_for_exit(pid, 5):
            _signal_process(pid, signal.SIGKILL)
            _wait_for_exit(pid, 1)

    _stop_runtime_fallback()
    paths["pid"].unlink(missing_ok=True)
    print("[safe-rehearsal] stopped")
    return 0


def main() -> int:
    args = parse_args()
    try:
        if args.command == "start":
            return _start(args)
        if args.command == "status":
            return _status(args)
        return _stop(args)
    except Exception as exc:
        print(f"[safe-rehearsal] ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
