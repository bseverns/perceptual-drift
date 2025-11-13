# OSC→MSP bridge telemetry cheat sheet

The control bridge already tattles on itself; this page explains how to read the
trail so ops, safety, and privacy folks can tell if a run stayed within the
ritual.

## Where the JSONL lives

* Default path: `logs/ops_events.jsonl` in the repo root.
* Override with `$PERCEPTUAL_DRIFT_LOG_DIR` if you want logs elsewhere (relative
  paths resolve inside the repo; absolute paths write wherever you point).
* Each event is a single JSON line containing ISO8601 `timestamp`, `operator`,
  `host`, and action-specific fields.

Keep the file append-only during a run. After wrap, sign it with
`scripts/seal_logs.sh` so the audit trail gets a tamper-evident bow.

## Event field glossary

| Field | Meaning |
| --- | --- |
| `action` | Short code for what just happened. See the catalog below. |
| `status` | Severity or state (`info`, `armed`, `neutralized`, `closed`, etc.). |
| `message` | Human-readable summary. Present for high-signal events. |
| `details` | Structured extras: file paths, intent strings, OSC addresses. |

## Event catalog

| Action | When it fires | Typical status values | Notes |
| --- | --- | --- | --- |
| `mapping_load` | Base YAML at `--config` loads | `info`, `error` | `details.path` holds the resolved file. |
| `recipe_load` | Recipe pulled in via `--recipe` | `info`, `error` | Includes `intent`, `slug`, and requested OSC port if present. |
| `osc_port_override` | OSC port deviates from default | `info` | `details.source` is `recipe` or `cli`. |
| `osc_bridge_boot` | OSC server starts listening | `info` | `message` contains host/port tuple. |
| `consent_toggle` | Crowd flips the consent gate | `armed`, `disarmed` | `details.value` is `1` or `0`; `details.osc_addr` shows the address that triggered it. |
| `osc_bridge_stream` | MSP writer changes posture | `armed`, `neutralized` | Fired when consent toggles or drops mid-run. Message clarifies why. |
| `osc_bridge_shutdown` | Bridge stops | `info`, `closed` | First event notes operator-triggered exit; second confirms serial port closed. |

## Suggested rotation & retention

* Treat each rehearsal/show as a session. Copy the JSONL plus the Minisign
  signature into a dated folder under `logs/archive/<yyyy-mm-dd_run-name>/`.
* Keep at least the last three runs locally so operators can diff behavior.
  Ship longer archives (and signatures) to your secure storage if privacy policy
  demands.
* If the file exceeds ~10 MB, roll it by renaming to `ops_events.jsonl.1` before
  the next run. Minisign each chunk separately.

## Fast queries

* Most recent consent gate flip:
  ```bash
  jq 'select(.action=="consent_toggle") | .timestamp, .status' logs/ops_events.jsonl | tail -n2
  ```
* Confirm a recipe was used:
  ```bash
  jq 'select(.action=="recipe_load") | {intent: .details.intent, slug: .details.slug}' logs/ops_events.jsonl
  ```
* Verify shutdown was graceful:
  ```bash
  jq 'select(.action=="osc_bridge_shutdown") | .status' logs/ops_events.jsonl
  ```

## Why bother?

During a safety audit, these records prove:

1. The bridge only streamed MSP frames while consent was armed.
2. Operators loaded the recipe they claimed (intent + slug are recorded).
3. Serial links were closed cleanly, so no phantom packets leaked once the cage
   went dark.

When something goes sideways, diff two sessions’ logs and look for missing or
unexpected events before diving into firmware rabbit holes.
