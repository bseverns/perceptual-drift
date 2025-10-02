#!/usr/bin/env bash
set -euo pipefail

RUNTIME_ROOT="${PERCEPTUAL_DRIFT_RUNTIME:-$HOME/perceptual-drift-runtime}"
TARGETS=(
  "$RUNTIME_ROOT/frame_cache"
  "$RUNTIME_ROOT/osc_ringbuffer"
  "$RUNTIME_ROOT/replay_exports"
  "./software/video-pipeline/tmp"
)

printf "[purge] scanning runtime caches...\n"
for dir in "${TARGETS[@]}"; do
  if [ -d "$dir" ]; then
    printf "[purge] clearing %s\n" "$dir"
    find "$dir" -mindepth 1 -print -exec rm -rf {} +
  else
    printf "[purge] %s not found, creating placeholder.\n" "$dir"
    mkdir -p "$dir"
  fi
  chmod 700 "$dir"
done

mkdir -p logs
log_line="$(date --iso-8601=seconds) | purge_buffers | $(whoami) | ${HOSTNAME:-unknown_host}"
printf "%s\n" "$log_line" >> logs/privacy_audit.log
printf "[purge] buffers nuked. audit trail appended to logs/privacy_audit.log\n"
