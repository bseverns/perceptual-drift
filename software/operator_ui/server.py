#!/usr/bin/env python3
"""Minimal operator web UI service for recipe/control visibility."""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import secrets
import sys
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Tuple
from urllib.parse import parse_qs, urlparse

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

from software.operator_ui.state import OperatorState
from software.operator_ui.supervisor import (
    PreflightRunner,
    StarterSupervisor,
    get_rehearsal_profile,
    list_rehearsal_profiles,
)


def _read_json_body(handler: BaseHTTPRequestHandler) -> Dict[str, Any]:
    length = int(handler.headers.get("Content-Length", "0"))
    if length <= 0:
        return {}
    raw = handler.rfile.read(length)
    if not raw:
        return {}
    return json.loads(raw.decode("utf-8"))


def _json_response(
    handler: BaseHTTPRequestHandler, payload: Dict[str, Any], status: int = 200
) -> None:
    body = json.dumps(payload).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json")
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(body)


def _error_response(handler: BaseHTTPRequestHandler, message: str, status: int) -> None:
    _json_response(handler, {"ok": False, "error": message}, status=status)


def _resolve_static(root: Path, requested: str) -> Tuple[Path, str]:
    if requested in {"", "/"}:
        rel = "index.html"
    else:
        rel = requested.lstrip("/")
    target = (root / rel).resolve()
    if not str(target).startswith(str(root.resolve())):
        raise FileNotFoundError("path escape")
    return target, rel


def make_handler(
    state: OperatorState,
    static_root: Path,
    supervisor: StarterSupervisor,
    preflight_runner: PreflightRunner,
    api_token: str = "",
):
    class OperatorHandler(BaseHTTPRequestHandler):
        server_version = "PerceptualDriftOperatorUI/0.1"

        _GET_HANDLERS = {
            "/api/health": "_api_get_health",
            "/api/state": "_api_get_state",
            "/api/recipes": "_api_get_recipes",
            "/api/mapping/curves": "_api_get_mapping_curves",
            "/api/runtime/health": "_api_get_runtime_health",
            "/api/runtime/supervisor": "_api_get_runtime_supervisor",
            "/api/rehearsal/profiles": "_api_get_rehearsal_profiles",
            "/api/rehearsal/session": "_api_get_rehearsal_session",
            "/api/session/latest": "_api_get_session_latest",
        }

        _POST_HANDLERS = {
            "/api/recipe": "_api_post_recipe",
            "/api/consent": "_api_post_consent",
            "/api/session/export": "_api_post_session_export",
            "/api/runtime/start": "_api_post_runtime_start",
            "/api/runtime/stop": "_api_post_runtime_stop",
            "/api/rehearsal/preflight": "_api_post_rehearsal_preflight",
            "/api/rehearsal/start": "_api_post_rehearsal_start",
            "/api/rehearsal/stop": "_api_post_rehearsal_stop",
        }

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path.startswith("/api/"):
                self._handle_api_get(parsed)
                return
            self._handle_static(parsed.path)

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            if not parsed.path.startswith("/api/"):
                _error_response(
                    self, "POST only allowed for /api endpoints", HTTPStatus.NOT_FOUND
                )
                return

            if api_token:
                auth_header = self.headers.get("Authorization", "")
                if not auth_header.startswith("Bearer ") or not secrets.compare_digest(auth_header[7:], api_token):
                    _error_response(self, "unauthorized", HTTPStatus.UNAUTHORIZED)
                    return

            self._handle_api_post(parsed)

        def log_message(self, fmt: str, *args) -> None:  # quiet default handler
            return

        def _api_get_health(self, _parsed, _query) -> None:
            _json_response(self, {"ok": True, "service": "operator-ui"})

        def _api_get_state(self, _parsed, _query) -> None:
            _json_response(self, {"ok": True, "state": state.snapshot()})

        def _api_get_recipes(self, _parsed, _query) -> None:
            _json_response(self, {"ok": True, "recipes": state.list_recipes()})

        def _api_get_mapping_curves(self, _parsed, query) -> None:
            points_raw = query.get("points", ["101"])[0]
            try:
                points = int(points_raw)
            except ValueError:
                points = 101
            _json_response(
                self, {"ok": True, "data": state.mapping_curves(points=points)}
            )

        def _api_get_runtime_health(self, _parsed, _query) -> None:
            _json_response(self, {"ok": True, "runtime": state.runtime_health()})

        def _api_get_runtime_supervisor(self, _parsed, _query) -> None:
            _json_response(self, {"ok": True, "supervisor": supervisor.status()})

        def _api_get_rehearsal_profiles(self, _parsed, _query) -> None:
            _json_response(self, {"ok": True, "profiles": list_rehearsal_profiles()})

        def _api_get_rehearsal_session(self, _parsed, _query) -> None:
            _json_response(self, {"ok": True, "rehearsal": state.rehearsal_session()})

        def _api_get_session_latest(self, _parsed, _query) -> None:
            _json_response(self, {"ok": True, "session": state.latest_export()})

        def _handle_api_get(self, parsed) -> None:
            path = parsed.path
            method_name = self._GET_HANDLERS.get(path)
            if method_name:
                handler = getattr(self, method_name)
                handler(parsed, parse_qs(parsed.query))
            else:
                _error_response(self, f"unknown endpoint: {path}", HTTPStatus.NOT_FOUND)

        def _api_post_recipe(self, _parsed, payload) -> None:
            recipe = str(payload.get("recipe", "")).strip()
            try:
                snapshot = state.set_recipe(recipe)
            except Exception as exc:  # pragma: no cover - parse/load errors
                _error_response(self, f"failed to load recipe '{recipe}': {exc}", 400)
                return
            _json_response(self, {"ok": True, "state": snapshot})

        def _api_post_consent(self, _parsed, payload) -> None:
            if "consent" not in payload:
                _error_response(self, "missing 'consent' field", 400)
                return
            try:
                snapshot = state.set_consent(payload["consent"])
            except Exception:
                _error_response(self, "invalid consent value", 400)
                return
            _json_response(self, {"ok": True, "state": snapshot})

        def _api_post_session_export(self, _parsed, payload) -> None:
            label = str(payload.get("label", "")).strip()
            notes = str(payload.get("notes", "")).strip()
            try:
                session = state.export_session(label=label, notes=notes)
            except Exception as exc:
                _error_response(self, f"session export failed: {exc}", 500)
                return
            _json_response(self, {"ok": True, "session": session})

        def _api_post_runtime_start(self, _parsed, payload) -> None:
            try:
                sup = supervisor.start(payload)
            except Exception as exc:
                _error_response(self, f"failed to start runtime: {exc}", 400)
                return
            _json_response(self, {"ok": True, "supervisor": sup})

        def _api_post_runtime_stop(self, _parsed, _payload) -> None:
            try:
                sup = supervisor.stop()
            except Exception as exc:
                _error_response(self, f"failed to stop runtime: {exc}", 500)
                return
            _json_response(self, {"ok": True, "supervisor": sup})

        def _api_post_rehearsal_preflight(self, _parsed, payload) -> None:
            strict = bool(payload.get("strict", False))
            preflight = preflight_runner.run(strict=strict)
            rehearsal = state.set_rehearsal_preflight(preflight)
            _json_response(
                self,
                {
                    "ok": preflight["ok"],
                    "preflight": preflight,
                    "rehearsal": rehearsal,
                },
                status=200 if preflight["ok"] else 412,
            )

        def _api_post_rehearsal_start(self, _parsed, payload) -> None:
            profile_id = str(payload.get("profile_id", "safe_synthetic")).strip()
            notes = str(payload.get("notes", "")).strip()
            label = str(payload.get("label", "")).strip()
            if not label:
                label = time.strftime("rehearsal_%Y%m%d_%H%M%S", time.localtime())
            strict = bool(payload.get("strict_preflight", False))
            skip_preflight = bool(payload.get("skip_preflight", False))
            try:
                profile = get_rehearsal_profile(profile_id)
            except ValueError as exc:
                _error_response(self, str(exc), 400)
                return
            preflight = state.rehearsal_session().get("last_preflight", {})
            if not skip_preflight:
                preflight = preflight_runner.run(strict=strict)
                state.set_rehearsal_preflight(preflight)
                if not preflight.get("ok", False):
                    _json_response(
                        self,
                        {
                            "ok": False,
                            "error": "preflight failed; rehearsal start blocked",
                            "preflight": preflight,
                        },
                        status=412,
                    )
                    return
            try:
                sup = supervisor.start(profile.get("start_options", {}))
            except Exception as exc:
                _error_response(self, f"failed to start runtime: {exc}", 400)
                return
            rehearsal = state.start_rehearsal(
                label=label,
                profile_id=profile_id,
                notes=notes,
                start_options=profile.get("start_options", {}),
                preflight=preflight,
            )
            _json_response(
                self,
                {
                    "ok": True,
                    "profile": profile,
                    "preflight": preflight,
                    "supervisor": sup,
                    "rehearsal": rehearsal,
                },
            )

        def _api_post_rehearsal_stop(self, _parsed, _payload) -> None:
            try:
                sup = supervisor.stop()
            except Exception as exc:
                _error_response(self, f"failed to stop runtime: {exc}", 500)
                return
            rehearsal = state.stop_rehearsal()
            _json_response(
                self, {"ok": True, "supervisor": sup, "rehearsal": rehearsal}
            )

        def _handle_api_post(self, parsed) -> None:
            path = parsed.path
            method_name = self._POST_HANDLERS.get(path)
            if not method_name:
                _error_response(self, f"unknown endpoint: {path}", HTTPStatus.NOT_FOUND)
                return

            try:
                payload = _read_json_body(self)
            except json.JSONDecodeError:
                _error_response(self, "invalid JSON body", HTTPStatus.BAD_REQUEST)
                return

            handler = getattr(self, method_name)
            handler(parsed, payload)

        def _handle_static(self, requested_path: str) -> None:
            try:
                target, _rel = _resolve_static(static_root, requested_path)
            except FileNotFoundError:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            if not target.is_file():
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            body = target.read_bytes()
            ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return OperatorHandler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Perceptual Drift operator UI server")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host")
    parser.add_argument("--port", type=int, default=8088, help="Bind port")
    parser.add_argument(
        "--base-mapping", default="config/mapping.yaml", help="Base mapping path"
    )
    parser.add_argument("--recipes-dir", default="config/recipes", help="Recipes dir")
    parser.add_argument(
        "--static-dir",
        default="software/operator_ui/static",
        help="Static web assets path",
    )
    parser.add_argument(
        "--runtime-targets",
        default="127.0.0.1:9000,127.0.0.1:9010",
        help=(
            "Comma-separated host:port OSC targets for runtime dispatch "
            "(consent and recipe patches)."
        ),
    )
    parser.add_argument(
        "--recipe-route",
        default="/pd/patch",
        help="OSC route for runtime recipe patch messages.",
    )
    parser.add_argument(
        "--consent-route",
        default="/pd/consent",
        help="OSC route for runtime consent messages.",
    )
    parser.add_argument(
        "--session-export-dir",
        default="runtime/operator_ui_sessions",
        help="Directory where session export JSON files are written.",
    )
    parser.add_argument(
        "--telemetry-file",
        default="runtime/swarm_latency.json",
        help="Optional telemetry snapshot JSON to include in session exports.",
    )
    parser.add_argument(
        "--starter-script",
        default="scripts/starter_up.sh",
        help="Starter bundle launcher script used by runtime start/stop endpoints.",
    )
    parser.add_argument(
        "--starter-log",
        default="runtime/operator_ui/starter_supervisor.log",
        help="Log file for starter process launched via runtime start endpoint.",
    )
    parser.add_argument(
        "--starter-doctor-script",
        default="scripts/starter_doctor.sh",
        help="Preflight doctor script used by rehearsal preflight endpoint.",
    )
    parser.add_argument(
        "--api-token",
        default=os.environ.get("OPERATOR_API_TOKEN", ""),
        help="Require token for API POST endpoints (via Authorization: Bearer <token>)",
    )
    return parser.parse_args()


def _parse_runtime_targets(raw: str) -> list[tuple[str, int]]:
    targets: list[tuple[str, int]] = []
    for part in raw.split(","):
        text = part.strip()
        if not text:
            continue
        if ":" not in text:
            raise ValueError(f"invalid runtime target '{text}' (expected host:port)")
        host, port_raw = text.rsplit(":", 1)
        if not host:
            raise ValueError(f"invalid runtime target '{text}' (missing host)")
        try:
            port = int(port_raw)
        except ValueError as exc:
            raise ValueError(
                f"invalid runtime target '{text}' (bad port)"
            ) from exc
        targets.append((host, port))
    return targets


def main() -> int:
    args = parse_args()
    runtime_targets = _parse_runtime_targets(args.runtime_targets)

    api_token = args.api_token
    if not api_token:
        api_token = secrets.token_urlsafe(16)
        print(f"[operator-ui] WARNING: No API token provided. Generated temporary token: {api_token}")

    state = OperatorState(
        base_mapping_path=Path(args.base_mapping).resolve(),
        recipes_dir=Path(args.recipes_dir).resolve(),
        runtime_targets=runtime_targets,
        recipe_route=args.recipe_route,
        consent_route=args.consent_route,
        export_dir=Path(args.session_export_dir).resolve(),
        telemetry_snapshot_file=Path(args.telemetry_file).resolve(),
    )
    supervisor = StarterSupervisor(
        script_path=Path(args.starter_script).resolve(),
        log_path=Path(args.starter_log).resolve(),
        cwd=REPO_ROOT,
    )
    preflight_runner = PreflightRunner(
        script_path=Path(args.starter_doctor_script).resolve(),
        cwd=REPO_ROOT,
    )
    static_root = Path(args.static_dir).resolve()
    handler = make_handler(state, static_root, supervisor, preflight_runner, api_token)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"[operator-ui] serving http://{args.host}:{args.port}")
    print(
        "[operator-ui] endpoints: /api/state /api/recipes "
        "/api/recipe /api/consent /api/mapping/curves "
        "/api/runtime/health /api/runtime/supervisor "
        "/api/runtime/start /api/runtime/stop "
        "/api/rehearsal/profiles /api/rehearsal/session "
        "/api/rehearsal/preflight /api/rehearsal/start /api/rehearsal/stop "
        "/api/session/export /api/session/latest"
    )
    print(
        "[operator-ui] runtime targets: "
        + ", ".join(f"{h}:{p}" for h, p in runtime_targets)
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
