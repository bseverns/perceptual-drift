# Operator UI (M5 Alpha)

This is the first M5 web UI slice:

- recipe loading
- consent monitoring/toggling
- mapping curve visualization (altitude/lateral)
- session export bundles (active config + curves + dispatch history + telemetry snapshot)

## Run

From repo root:

```bash
python3 software/operator_ui/server.py --host 127.0.0.1 --port 8088
```

Open:

`http://127.0.0.1:8088`

## API

- `GET /api/health`
- `GET /api/state`
- `GET /api/recipes`
- `POST /api/recipe` body: `{ "recipe": "ambient" }` (`"base"` resets)
- `POST /api/consent` body: `{ "consent": 1 }`
- `GET /api/mapping/curves?points=121`
- `GET /api/runtime/health`
- `GET /api/runtime/supervisor`
- `POST /api/runtime/start`
- `POST /api/runtime/stop`
- `POST /api/session/export` body: `{ "label": "run_a", "notes": "optional" }`
- `GET /api/session/latest`

## Runtime dispatch wiring

The server now emits OSC control messages to runtime targets when you call
`POST /api/recipe` or `POST /api/consent`.

Default runtime targets:

- `127.0.0.1:9000` (control bridge)
- `127.0.0.1:9010` (swarm bridge/sim)

Override targets/routes:

```bash
python3 software/operator_ui/server.py \
  --runtime-targets 127.0.0.1:9000,127.0.0.1:9010 \
  --recipe-route /pd/patch \
  --consent-route /pd/consent
```

Note: selecting recipe `base` resets local state but does not emit a runtime
recipe patch.

## Session exports

Session exports are written to JSON files and include:

- current operator state
- active merged mapping
- dispatch history
- generated mapping curves
- optional telemetry snapshot JSON

Default paths:

- export dir: `runtime/operator_ui_sessions`
- telemetry snapshot input: `runtime/swarm_latency.json`

Override export paths:

```bash
python3 software/operator_ui/server.py \
  --session-export-dir runtime/operator_ui_sessions \
  --telemetry-file runtime/swarm_latency.json
```

## Runtime health visibility

`GET /api/runtime/health` reports status for key daemons (control bridge, tracker,
swarm demo) using:

- PID files (preferred, if present)
- process table scan fallback (`ps`)

Starter bundle launches now write PID files to `runtime/starter_bundle/`:

- `bridge.pid`
- `tracker.pid`
- `video.pid`

## Runtime start/stop controls

The UI can start and stop the starter bundle launcher via:

- `POST /api/runtime/start`
- `POST /api/runtime/stop`
- `GET /api/runtime/supervisor`

By default this runs:

- script: `scripts/starter_up.sh`
- log: `runtime/operator_ui/starter_supervisor.log`

Override with:

```bash
python3 software/operator_ui/server.py \
  --starter-script scripts/starter_up.sh \
  --starter-log runtime/operator_ui/starter_supervisor.log
```

## Scope notes

This alpha now emits live OSC control intents, session exports, runtime health visibility, and starter runtime start/stop controls, but it does not yet include:

- supervised restart policies, retries, and multi-service orchestration
