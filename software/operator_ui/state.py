"""State management and mapping-curve helpers for the operator UI."""

from __future__ import annotations

import copy
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml
from pythonosc import udp_client

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

from software.swarm.mapping_loader import load_mapping


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _apply_deadzone(value: float, deadzone: float) -> float:
    if deadzone <= 0.0:
        return value
    if deadzone >= 1.0:
        return 0.0
    if abs(value) <= deadzone:
        return 0.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return math.copysign(scaled, value)


def _expo(value: float, strength: float) -> float:
    return math.copysign(abs(value) ** (1.0 + strength), value)


def shape_axis(axis_cfg: Dict[str, Any], value: float) -> float:
    deadzone = float(axis_cfg.get("deadzone", 0.0))
    deadzone = _clamp(deadzone, 0.0, 0.99)
    shaped = _apply_deadzone(value, deadzone)
    curve = str(axis_cfg.get("curve", "linear")).lower()
    if curve == "expo":
        strength = float(axis_cfg.get("expo_strength", axis_cfg.get("expo", 0.5)))
        shaped = _expo(shaped, strength)
    return _clamp(shaped, -1.0, 1.0)


class OperatorState:
    """Holds current operator-facing state for the web UI."""

    def __init__(
        self,
        base_mapping_path: Path,
        recipes_dir: Path,
        runtime_targets: Optional[Sequence[Tuple[str, int]]] = None,
        recipe_route: str = "/pd/patch",
        consent_route: str = "/pd/consent",
        export_dir: Optional[Path] = None,
        telemetry_snapshot_file: Optional[Path] = None,
        dispatch_history_limit: int = 50,
        runtime_services: Optional[Sequence[Dict[str, Any]]] = None,
    ) -> None:
        self.base_mapping_path = base_mapping_path
        self.recipes_dir = recipes_dir
        self.runtime_targets = list(runtime_targets or [])
        self.recipe_route = recipe_route
        self.consent_route = consent_route
        self.export_dir = (
            export_dir
            if export_dir is not None
            else (REPO_ROOT / "runtime" / "operator_ui_sessions")
        )
        self.telemetry_snapshot_file = telemetry_snapshot_file
        self.dispatch_history_limit = max(1, int(dispatch_history_limit))
        default_services = [
            {
                "id": "control_bridge",
                "name": "Control Bridge",
                "pid_file": str(REPO_ROOT / "runtime" / "starter_bundle" / "bridge.pid"),
                "match": "software/control-bridge/osc_msp_bridge.py",
            },
            {
                "id": "tracker",
                "name": "Tracker",
                "pid_file": str(REPO_ROOT / "runtime" / "starter_bundle" / "tracker.pid"),
                "match": "software/starter-bundle/minimal_tracker.py",
            },
            {
                "id": "swarm_demo",
                "name": "Swarm Demo",
                "pid_file": str(REPO_ROOT / "runtime" / "swarm_demo.pid"),
                "match": "software/swarm/swarm_demo.py",
            },
        ]
        self.runtime_services = [dict(s) for s in (runtime_services or default_services)]
        self._lock = threading.Lock()
        self._active_recipe: Optional[str] = None
        self._consent: int = 0
        self._updated_at: float = time.time()
        self._last_dispatch: Dict[str, Any] = {"action": "none", "results": []}
        self._dispatch_history: List[Dict[str, Any]] = []
        self._last_export_path: Optional[Path] = None
        self._mapping = load_mapping(base_path=str(self.base_mapping_path))

    def list_recipes(self) -> List[Dict[str, str]]:
        recipes: List[Dict[str, str]] = [
            {
                "id": "base",
                "name": "Base Mapping",
                "description": "Use config/mapping.yaml without recipe overlay.",
            }
        ]
        for path in sorted(self.recipes_dir.glob("*.yaml")):
            entry = {"id": path.stem, "name": path.stem, "description": ""}
            try:
                parsed = yaml.safe_load(path.read_text()) or {}
            except yaml.YAMLError:
                recipes.append(entry)
                continue
            if isinstance(parsed, dict):
                entry["name"] = str(parsed.get("name", path.stem))
                entry["description"] = str(parsed.get("description", "")).strip()
            recipes.append(entry)
        return recipes

    def set_recipe(self, recipe_id: str) -> Dict[str, Any]:
        normalized = (recipe_id or "").strip()
        if not normalized or normalized == "base":
            mapping = load_mapping(base_path=str(self.base_mapping_path))
            active = None
        else:
            mapping = load_mapping(
                base_path=str(self.base_mapping_path),
                recipe_name=normalized,
                recipes_dir=str(self.recipes_dir),
            )
            active = normalized

        with self._lock:
            self._mapping = mapping
            self._active_recipe = active
            self._updated_at = time.time()
            if normalized and normalized != "base":
                self._last_dispatch = self._emit_runtime(
                    action="recipe",
                    route=self.recipe_route,
                    payload=normalized,
                )
            else:
                self._last_dispatch = {
                    "action": "recipe",
                    "route": self.recipe_route,
                    "results": [],
                    "note": "base mapping selected; no runtime patch emitted",
                }
            self._record_dispatch(self._last_dispatch)
            return self.snapshot_unlocked()

    def set_consent(self, consent_value: Any) -> Dict[str, Any]:
        consent = 1 if float(consent_value) >= 0.5 else 0
        with self._lock:
            self._consent = consent
            self._updated_at = time.time()
            self._last_dispatch = self._emit_runtime(
                action="consent",
                route=self.consent_route,
                payload=consent,
            )
            self._record_dispatch(self._last_dispatch)
            return self.snapshot_unlocked()

    def _emit_runtime(self, action: str, route: str, payload: Any) -> Dict[str, Any]:
        results: List[Dict[str, Any]] = []
        for host, port in self.runtime_targets:
            entry = {"host": host, "port": port, "ok": True}
            try:
                client = udp_client.SimpleUDPClient(host, int(port))
                client.send_message(route, payload)
            except Exception as exc:
                entry["ok"] = False
                entry["error"] = str(exc)
            results.append(entry)
        return {
            "action": action,
            "route": route,
            "payload": payload,
            "results": results,
            "targets": len(self.runtime_targets),
            "sent_at": time.time(),
        }

    def _record_dispatch(self, event: Dict[str, Any]) -> None:
        self._dispatch_history.append(copy.deepcopy(event))
        if len(self._dispatch_history) > self.dispatch_history_limit:
            overflow = len(self._dispatch_history) - self.dispatch_history_limit
            if overflow > 0:
                del self._dispatch_history[:overflow]

    def _load_telemetry_snapshot(self) -> Dict[str, Any]:
        if self.telemetry_snapshot_file is None:
            return {"ok": False, "reason": "not_configured"}
        path = Path(self.telemetry_snapshot_file)
        if not path.is_file():
            return {"ok": False, "reason": "missing_file", "path": str(path)}
        try:
            data = json.loads(path.read_text())
        except json.JSONDecodeError as exc:
            return {
                "ok": False,
                "reason": "invalid_json",
                "path": str(path),
                "error": str(exc),
            }
        return {"ok": True, "path": str(path), "data": data}

    def export_session(
        self, *, label: str = "", notes: str = ""
    ) -> Dict[str, Any]:
        safe_label = "".join(ch for ch in label.strip() if ch.isalnum() or ch in "-_")
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        filename = (
            f"session_{timestamp}_{safe_label}.json"
            if safe_label
            else f"session_{timestamp}.json"
        )
        out_dir = Path(self.export_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / filename

        with self._lock:
            mapping_copy = copy.deepcopy(self._mapping)
            mapping_section = dict(mapping_copy.get("mapping", {}))
            payload = {
                "exported_at": time.time(),
                "label": label,
                "notes": notes,
                "state": self.snapshot_unlocked(),
                "mapping": mapping_copy,
                "dispatch_history": copy.deepcopy(self._dispatch_history),
                "curves": self._mapping_curves_from_mapping(mapping_section, 81),
                "telemetry_snapshot": self._load_telemetry_snapshot(),
            }

        out_path.write_text(json.dumps(payload, indent=2) + "\n")
        with self._lock:
            self._last_export_path = out_path

        return {
            "path": str(out_path),
            "bytes": out_path.stat().st_size,
            "label": label,
            "exported_at": payload["exported_at"],
        }

    def latest_export(self) -> Dict[str, Any]:
        with self._lock:
            path = self._last_export_path
        if path is None:
            return {"ok": False, "reason": "no_exports_yet"}
        exists = path.is_file()
        return {
            "ok": exists,
            "path": str(path),
            "exists": exists,
            "bytes": path.stat().st_size if exists else 0,
        }

    @staticmethod
    def _pid_running(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _ps_processes(self) -> List[Tuple[int, str]]:
        try:
            proc = subprocess.run(
                ["ps", "-ax", "-o", "pid=", "-o", "command="],
                capture_output=True,
                text=True,
                check=True,
            )
        except Exception:
            return []
        processes: List[Tuple[int, str]] = []
        for line in proc.stdout.splitlines():
            raw = line.strip()
            if not raw:
                continue
            parts = raw.split(None, 1)
            if not parts:
                continue
            try:
                pid = int(parts[0])
            except ValueError:
                continue
            command = parts[1] if len(parts) > 1 else ""
            processes.append((pid, command))
        return processes

    def runtime_health(self) -> Dict[str, Any]:
        processes = self._ps_processes()
        services: List[Dict[str, Any]] = []
        healthy = 0
        for raw_service in self.runtime_services:
            service = dict(raw_service)
            sid = str(service.get("id", "service"))
            name = str(service.get("name", sid))
            pid_file_raw = str(service.get("pid_file", "")).strip()
            match = str(service.get("match", "")).strip()

            pid: Optional[int] = None
            source = "none"
            detail = "not running"
            is_healthy = False

            if pid_file_raw:
                pid_path = Path(pid_file_raw)
                if pid_path.is_file():
                    try:
                        pid = int(pid_path.read_text().strip())
                        if self._pid_running(pid):
                            is_healthy = True
                            source = "pid_file"
                            detail = f"pid {pid} from {pid_path.name}"
                    except ValueError:
                        detail = f"invalid pid file: {pid_path.name}"

            if not is_healthy and match:
                found = [(p, cmd) for p, cmd in processes if match in cmd]
                if found:
                    pid = found[0][0]
                    is_healthy = True
                    source = "process_scan"
                    detail = f"matched '{match}' ({len(found)} process{'es' if len(found) != 1 else ''})"

            if is_healthy:
                healthy += 1

            services.append(
                {
                    "id": sid,
                    "name": name,
                    "healthy": is_healthy,
                    "pid": pid or 0,
                    "source": source,
                    "detail": detail,
                    "pid_file": pid_file_raw,
                    "match": match,
                }
            )

        return {
            "checked_at": time.time(),
            "healthy": healthy,
            "total": len(services),
            "services": services,
        }

    def mapping_curves(self, points: int = 101) -> Dict[str, Any]:
        points = int(_clamp(points, 5, 401))
        with self._lock:
            mapping = dict(self._mapping.get("mapping", {}))
        return self._mapping_curves_from_mapping(mapping, points)

    def _mapping_curves_from_mapping(
        self, mapping: Dict[str, Any], points: int
    ) -> Dict[str, Any]:
        points = int(_clamp(points, 5, 401))
        x_values = [(-1.0 + 2.0 * idx / (points - 1)) for idx in range(points)]
        curves = {}
        for axis_name in ("altitude", "lateral"):
            axis_cfg = mapping.get(axis_name, {}) or {}
            gain = float(axis_cfg.get("gain", 1.0))
            series = []
            for x in x_values:
                shaped = shape_axis(axis_cfg, x)
                series.append({"x": x, "y": _clamp(shaped * gain, -1.0, 1.0)})
            curves[axis_name] = {
                "deadzone": float(axis_cfg.get("deadzone", 0.0)),
                "curve": str(axis_cfg.get("curve", "linear")),
                "gain": gain,
                "series": series,
            }

        yaw_cfg = mapping.get("yaw_bias", {}) or {}
        return {
            "curves": curves,
            "yaw_bias": {
                "bias": float(yaw_cfg.get("bias", 0.0)),
                "jitter": float(yaw_cfg.get("jitter", 0.0)),
            },
            "glitch_intensity": mapping.get("glitch_intensity", {}),
            "led_palette": mapping.get("leds", {}).get("palette", []),
        }

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return self.snapshot_unlocked()

    def snapshot_unlocked(self) -> Dict[str, Any]:
        osc_cfg = self._mapping.get("osc", {}) if isinstance(self._mapping, dict) else {}
        consent_cfg = (
            self._mapping.get("consent", {}) if isinstance(self._mapping, dict) else {}
        )
        return {
            "active_recipe": self._active_recipe or "base",
            "consent_state": self._consent,
            "updated_at": self._updated_at,
            "osc_port": int(osc_cfg.get("port", 9000)),
            "consent_mode": str(consent_cfg.get("mode", "binary")),
            "consent_gate_motion": bool(consent_cfg.get("gate_motion", True)),
            "runtime_targets": [
                f"{host}:{port}" for host, port in self.runtime_targets
            ],
            "last_dispatch": self._last_dispatch,
            "dispatch_events": len(self._dispatch_history),
            "last_export": str(self._last_export_path) if self._last_export_path else "",
        }
