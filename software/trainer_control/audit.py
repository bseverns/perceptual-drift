"""Structured operational audit events for the trainer runtime."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping, Optional


REPO_ROOT = Path(__file__).resolve().parents[2]


class AuditLogger:
    """Write the same JSONL event schema used by the legacy control bridge."""

    def __init__(self, log_path: Optional[Path] = None) -> None:
        if log_path is None:
            configured = os.environ.get("PERCEPTUAL_DRIFT_LOG_DIR")
            log_dir = Path(configured) if configured else REPO_ROOT / "logs"
            if not log_dir.is_absolute():
                log_dir = REPO_ROOT / log_dir
            log_path = log_dir / "ops_events.jsonl"
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.operator = (
            os.environ.get("OPERATOR_ID")
            or os.environ.get("USER")
            or os.environ.get("USERNAME")
            or "unknown"
        )
        self.host = os.environ.get("HOSTNAME", "unknown_host")

    def write(
        self,
        action: str,
        status: str = "info",
        message: Optional[str] = None,
        details: Optional[Mapping] = None,
    ) -> None:
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
            event["details"] = dict(details)
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(event, ensure_ascii=False) + "\n")
