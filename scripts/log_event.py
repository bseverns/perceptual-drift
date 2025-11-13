#!/usr/bin/env python3
"""Write a structured audit event into ``logs/ops_events.jsonl``.

This consolidates the inline Python snippets that shell helpers used to keep
around. Pass action/status/message as positional args and optionally a JSON
blob for ``details``. If ``details`` is ``-`` we slurp JSON from stdin so
callers can pipe richer structures without shell escaping gymnastics.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional


@dataclass
class Event:
    action: str
    status: str
    message: str
    details: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "operator": os.environ.get("OPERATOR_ID")
            or os.environ.get("USER")
            or os.environ.get("USERNAME")
            or "unknown",
            "host": os.environ.get("HOSTNAME", "unknown_host"),
            "action": self.action,
            "status": self.status,
            "message": self.message,
        }
        if self.details is not None:
            payload["details"] = self.details
        return payload


def resolve_log_path() -> Path:
    repo_root = Path(__file__).resolve().parents[1]
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
    return log_dir / "ops_events.jsonl"


def load_details(raw: Optional[str]) -> Optional[Dict[str, Any]]:
    if raw is None:
        return None
    if raw == "-":
        raw = sys.stdin.read()
    raw = raw.strip()
    if not raw:
        return None
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {"raw": raw}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Append an ops audit event.")
    parser.add_argument("action", help="Short slug like 'purge_buffers' or 'record_fpv'.")
    parser.add_argument("status", help="info|success|error|warning — anything you'd log in AuditLogger.")
    parser.add_argument("message", help="Human-readable summary for future investigators.")
    parser.add_argument(
        "details",
        nargs="?",
        default=None,
        help="Optional JSON string or '-' to read JSON from stdin.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    details = load_details(args.details)
    event = Event(action=args.action, status=args.status, message=args.message, details=details)
    log_path = resolve_log_path()
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(event.to_dict(), ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
