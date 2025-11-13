#!/usr/bin/env bash
set -euo pipefail

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


if ! command -v minisign >/dev/null 2>&1; then
  log_event "seal_logs" "error" "minisign is not installed."
  echo "[seal] minisign binary not found in PATH." >&2
  exit 1
fi

if [ ! -d "$PERCEPTUAL_DRIFT_LOG_DIR" ]; then
  log_event "seal_logs" "error" "Log directory not found."
  echo "[seal] log directory $PERCEPTUAL_DRIFT_LOG_DIR is missing." >&2
  exit 1
fi

# Bail if there are no files to bundle aside from previous bundles/signatures.
if ! find "$PERCEPTUAL_DRIFT_LOG_DIR" -mindepth 1 -maxdepth 1 \( -type f -o -type d \) \
  ! -name 'bundles' ! -name '*.tar.gz' ! -name '*.minisig' \
  | grep -q .; then
  log_event "seal_logs" "skipped" "No new log artifacts to seal."
  echo "[seal] nothing to package; skipping." >&2
  exit 0
fi

secret_key="${MINISIGN_SECRET_KEY:-$HOME/.minisign/minisign.key}"
if [ ! -f "$secret_key" ]; then
  log_event "seal_logs" "error" "Minisign secret key missing."
  echo "[seal] minisign secret key not found at $secret_key" >&2
  exit 1
fi

mkdir -p "$PERCEPTUAL_DRIFT_LOG_DIR/bundles"
timestamp=$(date +%Y%m%d_%H%M%S)
bundle_path="$PERCEPTUAL_DRIFT_LOG_DIR/bundles/privacy_audit_${timestamp}.tar.gz"

# Package logs excluding previous bundles and signatures to avoid recursion.
tar -czf "$bundle_path" -C "$PERCEPTUAL_DRIFT_LOG_DIR" \
  --exclude "bundles" --exclude "*.tar.gz" --exclude "*.minisig" .

declare -a minisign_args=("-S" "-s" "$secret_key" "-m" "$bundle_path")
trusted_comment="${MINISIGN_TRUSTED_COMMENT:-Perceptual Drift log bundle}" 
if [ -n "$trusted_comment" ]; then
  minisign_args+=("-t" "$trusted_comment")
fi

if [ -n "${MINISIGN_UNTRUSTED_COMMENT:-}" ]; then
  minisign_args+=("-c" "$MINISIGN_UNTRUSTED_COMMENT")
fi

minisign "${minisign_args[@]}"

export SEALED_BUNDLE="$bundle_path"
DETAILS=$(python3 <<'PY'
import json
import os

print(json.dumps({
    "bundle": os.environ.get("SEALED_BUNDLE"),
    "trusted_comment": os.environ.get("MINISIGN_TRUSTED_COMMENT", "Perceptual Drift log bundle"),
}))
PY
)
log_event "seal_logs" "success" "Log bundle sealed with minisign." "$DETAILS"
unset SEALED_BUNDLE DETAILS

echo "[seal] bundle ready → $bundle_path" >&2
