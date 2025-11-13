#!/usr/bin/env bash
set -euo pipefail

RUNTIME_ROOT="${PERCEPTUAL_DRIFT_RUNTIME:-$HOME/perceptual-drift-runtime}"
TARGETS=(
  "$RUNTIME_ROOT/frame_cache"
  "$RUNTIME_ROOT/osc_ringbuffer"
  "$RUNTIME_ROOT/replay_exports"
  "./software/video-pipeline/tmp"
)

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_HELPER="$SCRIPT_DIR/log_event.py"
export PERCEPTUAL_DRIFT_LOG_DIR="${PERCEPTUAL_DRIFT_LOG_DIR:-$REPO_ROOT/logs}"

log_event() {
  local action="$1"
  local status="$2"
  local message="$3"
  local details_json="${4:-}"

  if [ -n "$details_json" ]; then
    python3 "$LOG_HELPER" "$action" "$status" "$message" "$details_json"
  else
    python3 "$LOG_HELPER" "$action" "$status" "$message"
  fi
}


printf "[purge] scanning runtime caches...\n"
purged_dirs=()
created_dirs=()
for dir in "${TARGETS[@]}"; do
  if [ -d "$dir" ]; then
    printf "[purge] clearing %s\n" "$dir"
    find "$dir" -mindepth 1 -print -exec rm -rf {} +
    purged_dirs+=("$dir")
  else
    printf "[purge] %s not found, creating placeholder.\n" "$dir"
    mkdir -p "$dir"
    created_dirs+=("$dir")
  fi
  chmod 700 "$dir"
done

export PURGED_DIRS="$(printf '%s\n' "${purged_dirs[@]-}")"
export CREATED_DIRS="$(printf '%s\n' "${created_dirs[@]-}")"
AUDIT_DETAILS=$(python3 <<'PY'
import json
import os

purged = [line for line in os.environ.get("PURGED_DIRS", "").splitlines() if line]
created = [line for line in os.environ.get("CREATED_DIRS", "").splitlines() if line]
print(json.dumps({
    "purged": purged,
    "created": created,
}))
PY
)
unset PURGED_DIRS CREATED_DIRS

log_event "purge_buffers" "success" "Runtime caches cleared." "$AUDIT_DETAILS"
unset AUDIT_DETAILS

printf "[purge] buffers nuked. audit trail appended to logs/ops_events.jsonl\n"
