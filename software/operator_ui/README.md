# Operator UI (M5 Alpha)

This is the first M5 web UI slice:

- recipe loading
- consent monitoring/toggling
- mapping curve visualization (altitude/lateral)

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

## Scope notes

This alpha now emits live OSC control intents, but it does not yet include:

- session export bundles (config + telemetry snapshot)
- runtime process supervision/health checks for bridge/sim daemons
