import json
import threading
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Iterator, Tuple
from urllib import error, request

import pytest

from software.operator_ui.server import (
    _parse_runtime_targets,
    _resolve_static,
    make_handler,
)
from software.operator_ui.state import OperatorState

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("port", ["0", "-1", "65536"])
def test_runtime_targets_reject_impossible_udp_ports(port):
    with pytest.raises(ValueError, match="port must be in range 1..65535"):
        _parse_runtime_targets(f"127.0.0.1:{port}")


def test_runtime_targets_accept_udp_port_boundaries():
    assert _parse_runtime_targets("127.0.0.1:1,localhost:65535") == [
        ("127.0.0.1", 1),
        ("localhost", 65535),
    ]


class _DummySupervisor:
    def status(self) -> Dict[str, object]:
        return {
            "running": False,
            "pid": 0,
            "started_at": 0.0,
            "args": [],
            "log_path": "",
            "last_exit_code": 0,
            "last_error": "",
        }


class _DummyPreflightRunner:
    def run(self, *, strict: bool = False) -> Dict[str, object]:
        return {
            "ok": True,
            "required_failures": 0,
            "warnings": 0,
            "strict": strict,
            "checked_at": 0.0,
            "checks": [],
        }


def _request_json(
    base_url: str,
    path: str,
    *,
    method: str = "GET",
    token: str = "",
    payload: Dict[str, object] | None = None,
) -> Tuple[int, Dict[str, object]]:
    headers: Dict[str, str] = {}
    body = None
    if token:
        headers["Authorization"] = f"Bearer {token}"
    if payload is not None:
        headers["Content-Type"] = "application/json"
        body = json.dumps(payload).encode("utf-8")

    req = request.Request(
        f"{base_url}{path}",
        data=body,
        headers=headers,
        method=method,
    )
    try:
        with request.urlopen(req, timeout=5) as resp:
            parsed = json.loads(resp.read().decode("utf-8"))
            return resp.status, parsed
    except error.HTTPError as exc:
        parsed = json.loads(exc.read().decode("utf-8"))
        return exc.code, parsed


@contextmanager
def _serve(api_token: str = "test-token") -> Iterator[str]:
    state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
    )
    handler = make_handler(
        state,
        ROOT / "software" / "operator_ui" / "static",
        _DummySupervisor(),
        _DummyPreflightRunner(),
        api_token=api_token,
    )
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address
        yield f"http://{host}:{port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_health_endpoint_stays_public():
    with _serve() as base_url:
        status, payload = _request_json(base_url, "/api/health")
    assert status == 200
    assert payload["ok"] is True


def test_read_endpoint_requires_auth_token():
    with _serve(api_token="secret-token") as base_url:
        status, payload = _request_json(base_url, "/api/state")
        assert status == 401
        assert payload["ok"] is False

        status, payload = _request_json(
            base_url,
            "/api/state",
            token="secret-token",
        )
    assert status == 200
    assert payload["ok"] is True


def test_post_endpoint_requires_auth_token():
    with _serve(api_token="secret-token") as base_url:
        status, payload = _request_json(
            base_url,
            "/api/consent",
            method="POST",
            payload={"consent": 1},
        )
        assert status == 401
        assert payload["ok"] is False

        status, payload = _request_json(
            base_url,
            "/api/consent",
            method="POST",
            token="secret-token",
            payload={"consent": 1},
        )
    assert status == 200
    assert payload["ok"] is True
    assert payload["state"]["consent_state"] == 1


def test_static_path_resolution_rejects_sibling_prefix_escape(tmp_path):
    root = tmp_path / "static"
    sibling = tmp_path / "static_evil"
    root.mkdir()
    sibling.mkdir()
    (sibling / "secret.txt").write_text("leaked")

    with pytest.raises(FileNotFoundError):
        _resolve_static(root, "/../static_evil/secret.txt")
