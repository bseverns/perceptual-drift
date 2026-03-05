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

## Scope notes

This alpha holds local operator state only. It does not yet push recipe/consent
changes into running bridge/swarm processes automatically. M5 follow-up will
wire these endpoints to live runtime controllers.

