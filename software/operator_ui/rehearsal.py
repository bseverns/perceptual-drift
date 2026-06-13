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
DEFAULT_STARTER_RUNTIME_DIR = REPO_ROOT / "runtime" / "starter_bundle"
DEFAULT_START_OPTIONS: Dict[str, Any] = {
    "tracker_mode": "synthetic",
    "video": "off",
    "serial": "FAKE",
    "tracker_host": "127.0.0.1",
}


def _build_parser(*, status_only: bool = False) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Show local safe rehearsal status."
            if status_only
            else "Start or control a safe no-hardware rehearsal."
        )
    )
    if not status_only:
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
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    return _build_parser().parse_args(argv)


def parse_status_args(argv: list[str] | None = None) -> argparse.Namespace:
    args = _build_parser(status_only=True).parse_args(argv)
    args.command = "status"
    return args


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


def _pid_file_status(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {
            "pid": 0,
            "running": False,
            "state": "missing",
            "detail": "pid file missing",
            "pid_file": str(path),
        }
    try:
        pid = int(path.read_text().strip())
    except (OSError, ValueError):
        return {
            "pid": 0,
            "running": False,
            "state": "invalid",
            "detail": "invalid pid file",
            "pid_file": str(path),
        }
    if _pid_running(pid):
        return {
            "pid": pid,
            "running": True,
            "state": "running",
            "detail": f"running (pid {pid})",
            "pid_file": str(path),
        }
    return {
        "pid": pid,
        "running": False,
        "state": "stale",
        "detail": f"stale pid file (pid {pid})",
        "pid_file": str(path),
    }


def _starter_service_statuses(runtime_dir: Path) -> list[Dict[str, Any]]:
    services = []
    for sid, name in (
        ("bridge", "Bridge"),
        ("tracker", "Tracker"),
        ("video", "Video"),
    ):
        pid_path = runtime_dir / f"{sid}.pid"
        info = _pid_file_status(pid_path)
        services.append(
            {
                "id": sid,
                "name": name,
                "healthy": bool(info["running"]),
                "pid": int(info["pid"]),
                "source": "pid_file" if info["state"] == "running" else "none",
                "detail": str(info["detail"]),
                "pid_file": str(pid_path),
                "match": "",
                "state": str(info["state"]),
            }
        )
    return services


def _parse_supervisor_args(argv: list[str]) -> Dict[str, Any]:
    parsed: Dict[str, Any] = dict(DEFAULT_START_OPTIONS)
    parsed["osc_port"] = 9000
    parsed["hz"] = 30
    parsed["recipe"] = "base"
    idx = 0
    while idx < len(argv):
        key = argv[idx]
        if key in {
            "--tracker-mode",
            "--video",
            "--serial",
            "--tracker-host",
            "--osc-port",
            "--hz",
            "--video-device",
            "--video-preset",
            "--recipe",
        } and idx + 1 < len(argv):
            raw = argv[idx + 1]
            idx += 2
            if key == "--tracker-mode":
                parsed["tracker_mode"] = raw
            elif key == "--video":
                parsed["video"] = raw
            elif key == "--serial":
                parsed["serial"] = raw
            elif key == "--tracker-host":
                parsed["tracker_host"] = raw
            elif key == "--osc-port":
                parsed["osc_port"] = int(raw)
            elif key == "--hz":
                parsed["hz"] = int(raw)
            elif key == "--video-device":
                parsed["video_device"] = raw
            elif key == "--video-preset":
                parsed["video_preset"] = raw
            elif key == "--recipe":
                parsed["recipe"] = raw
            continue
        idx += 1

    recipe = str(parsed.get("recipe", "base")).strip()
    parsed["recipe_name"] = (
        Path(recipe).stem if recipe not in {"", "base"} else "base"
    )
    parsed["mode"] = (
        "dry-run" if parsed.get("serial") == "FAKE" else "hardware"
    )
    return parsed


def _consent_label(value: Any) -> str:
    if value is None:
        return "unknown"
    return "ON" if float(value) >= 0.5 else "OFF"


def _service_line(service: Dict[str, Any]) -> str:
    return (
        f"{service['id']}: running (pid {service['pid']})"
        if service.get("healthy")
        else f"{service['id']}: {service['detail']}"
    )


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
    ui_pid = _pid_file_status(paths["pid"])
    starter_runtime_dir = Path(
        meta.get("starter_runtime_dir", DEFAULT_STARTER_RUNTIME_DIR)
    ).resolve()
    starter_services = _starter_service_statuses(starter_runtime_dir)
    starter_running = any(service["healthy"] for service in starter_services)
    status: Dict[str, Any] = {
        "operator_ui_running": bool(ui_pid["running"]),
        "operator_ui_pid": int(ui_pid["pid"]) if ui_pid["running"] else 0,
        "operator_ui_pid_state": ui_pid["state"],
        "operator_ui_detail": ui_pid["detail"],
        "base_url": base_url,
        "token_path": str(paths["token"]),
        "operator_ui_log": str(paths["log"]),
        "starter_log": str(paths["starter_log"]),
        "runtime_dir": str(runtime_dir),
        "starter_runtime_dir": str(starter_runtime_dir),
        "runtime_running": starter_running,
        "runtime_pid": 0,
        "consent_state": None,
        "active_recipe": "base",
        "runtime_targets": [],
        "osc_port": 9000,
        "mode": "dry-run",
        "supervisor_args": [],
        "services": starter_services,
        "stop_command": "pd-safe-rehearsal stop",
    }
    if not ui_pid["running"]:
        return status
    try:
        supervisor = _api_request(
            base_url, "/api/runtime/supervisor", token=token
        )
        state = _api_request(base_url, "/api/state", token=token)
        runtime = _api_request(base_url, "/api/runtime/health", token=token)
    except Exception:
        return status
    sup = supervisor.get("supervisor", {})
    runtime_state = state.get("state", {})
    runtime_health = runtime.get("runtime", {})
    start_options = _parse_supervisor_args(list(sup.get("args", [])))
    status["runtime_running"] = bool(sup.get("running", False))
    status["runtime_pid"] = int(sup.get("pid", 0) or 0)
    status["consent_state"] = runtime_state.get("consent_state")
    status["active_recipe"] = str(
        runtime_state.get("active_recipe", start_options["recipe_name"])
    )
    status["runtime_targets"] = list(runtime_state.get("runtime_targets", []))
    status["osc_port"] = int(runtime_state.get("osc_port", 9000))
    status["mode"] = str(start_options.get("mode", "dry-run"))
    status["supervisor_args"] = list(sup.get("args", []))
    status["services"] = list(runtime_health.get("services", starter_services))
    status["starter_log"] = str(sup.get("log_path", status["starter_log"]))
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
        f"running (pid {status['operator_ui_pid']}) at {status['base_url']}"
        if status["operator_ui_running"]
        else status["operator_ui_detail"]
    )
    runtime_state = (
        f"running (pid {status['runtime_pid']})"
        if status["runtime_running"] and status["runtime_pid"]
        else ("running" if status["runtime_running"] else "stopped")
    )
    print(f"[pd-status] operator UI: {ui_state}")
    print(f"[pd-status] starter runtime: {runtime_state}")
    print(f"[pd-status] recipe: {status['active_recipe']}")
    print(f"[pd-status] consent: {_consent_label(status['consent_state'])}")
    print(f"[pd-status] mode: {status['mode']}")
    print(
        "[pd-status] ports: "
        f"operator UI {status['base_url']}, bridge OSC udp/{status['osc_port']}"
    )
    for service in status["services"]:
        print(f"[pd-status] {_service_line(service)}")
    print(
        "[pd-status] logs: "
        f"{status['operator_ui_log']}, {status['starter_log']}, "
        f"{status['starter_runtime_dir']}"
    )
    print(f"[pd-status] token file: {status['token_path']}")
    print(f"[pd-status] stop: {status['stop_command']}")
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


def status_main() -> int:
    args = parse_status_args()
    try:
        return _status(args)
    except Exception as exc:
        print(f"[pd-status] ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
