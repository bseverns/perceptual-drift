"""Audit/event logging helpers for the OSC->MSP bridge."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path


class AuditLogger:
    def __init__(self):
        repo_root = Path(__file__).resolve().parents[2]
        log_dir_env = os.environ.get("PERCEPTUAL_DRIFT_LOG_DIR")
        if log_dir_env:
            candidate = Path(log_dir_env)
            if candidate.is_absolute():
                log_dir = candidate
            else:
                log_dir = repo_root / candidate
        else:
            log_dir = repo_root / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / "ops_events.jsonl"
        self.operator = (
            os.environ.get("OPERATOR_ID")
            or os.environ.get("USER")
            or os.environ.get("USERNAME")
            or "unknown"
        )
        self.host = os.environ.get("HOSTNAME", "unknown_host")

    def write(self, action, status="info", message=None, details=None):
        event = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "operator": self.operator,
            "host": self.host,
            "action": action,
            "status": status,
        }
        if message:
            event["message"] = message
        if details is not None:
            event["details"] = details
        with self.log_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(event, ensure_ascii=False) + "\n")
