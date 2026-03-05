#!/usr/bin/env python3
"""Minimal operator web UI service for recipe/control visibility."""

from __future__ import annotations

import argparse
import json
import mimetypes
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Tuple
from urllib.parse import parse_qs, urlparse

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

from software.operator_ui.state import OperatorState


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


def make_handler(state: OperatorState, static_root: Path):
    class OperatorHandler(BaseHTTPRequestHandler):
        server_version = "PerceptualDriftOperatorUI/0.1"

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
            self._handle_api_post(parsed)

        def log_message(self, fmt: str, *args) -> None:  # quiet default handler
            return

        def _handle_api_get(self, parsed) -> None:
            path = parsed.path
            query = parse_qs(parsed.query)
            if path == "/api/health":
                _json_response(self, {"ok": True, "service": "operator-ui"})
                return
            if path == "/api/state":
                _json_response(self, {"ok": True, "state": state.snapshot()})
                return
            if path == "/api/recipes":
                _json_response(self, {"ok": True, "recipes": state.list_recipes()})
                return
            if path == "/api/mapping/curves":
                points_raw = query.get("points", ["101"])[0]
                try:
                    points = int(points_raw)
                except ValueError:
                    points = 101
                _json_response(
                    self, {"ok": True, "data": state.mapping_curves(points=points)}
                )
                return
            _error_response(self, f"unknown endpoint: {path}", HTTPStatus.NOT_FOUND)

        def _handle_api_post(self, parsed) -> None:
            path = parsed.path
            try:
                payload = _read_json_body(self)
            except json.JSONDecodeError:
                _error_response(self, "invalid JSON body", HTTPStatus.BAD_REQUEST)
                return
            if path == "/api/recipe":
                recipe = str(payload.get("recipe", "")).strip()
                try:
                    snapshot = state.set_recipe(recipe)
                except Exception as exc:  # pragma: no cover - parse/load errors
                    _error_response(
                        self, f"failed to load recipe '{recipe}': {exc}", 400
                    )
                    return
                _json_response(self, {"ok": True, "state": snapshot})
                return
            if path == "/api/consent":
                if "consent" not in payload:
                    _error_response(self, "missing 'consent' field", 400)
                    return
                try:
                    snapshot = state.set_consent(payload["consent"])
                except Exception:
                    _error_response(self, "invalid consent value", 400)
                    return
                _json_response(self, {"ok": True, "state": snapshot})
                return
            _error_response(self, f"unknown endpoint: {path}", HTTPStatus.NOT_FOUND)

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
    state = OperatorState(
        base_mapping_path=Path(args.base_mapping).resolve(),
        recipes_dir=Path(args.recipes_dir).resolve(),
        runtime_targets=runtime_targets,
        recipe_route=args.recipe_route,
        consent_route=args.consent_route,
    )
    static_root = Path(args.static_dir).resolve()
    handler = make_handler(state, static_root)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"[operator-ui] serving http://{args.host}:{args.port}")
    print(
        "[operator-ui] endpoints: /api/state /api/recipes "
        "/api/recipe /api/consent /api/mapping/curves"
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
