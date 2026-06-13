# Mapping Contract (Canonical Reference)

This document is the single source of truth for the control mapping schema used
by the tracker, bridge, and recipes.

Authoritative file:

- [`config/mapping.yaml`](../config/mapping.yaml)

## Purpose

The mapping contract defines:

1. OSC address expectations from gesture trackers.
2. Consent gating behavior for motion outputs.
3. Curve/gain shaping from normalized gesture signals to RC intent.
4. Bridge runtime behavior (rate, modes, stale watchdog).

## Top-level schema

`config/mapping.yaml` supports these top-level keys:

- `osc`
- `consent`
- `mapping`
- `bridge`

## `osc`

Defines OSC ingress port and address routes.

- `osc.port`: UDP port the bridge listens on.
- `osc.address_space.altitude`
- `osc.address_space.lateral`
- `osc.address_space.yaw`
- `osc.address_space.crowd`
- `osc.address_space.consent`

Trackers should publish normalized values to these routes:

- altitude: `0..1`
- lateral: `-1..1`
- yaw: `-1..1`
- crowd: `0..1`
- consent: `0|1` (or float interpreted by consent mode)

Recommended tracker semantics:

- `altitude`, `lateral`, and `yaw` should describe where the active participant
  mass is located in frame.
- `crowd` should describe recent motion intensity, not merely static occupancy.
- Trackers may change how they estimate those values internally as long as the
  published ranges and consent behavior stay stable.

## `consent`

Defines how participation state gates motion.

- `source`: OSC route used for consent input.
- `mode`: `binary` or `smooth`.
- `default_state`: fallback when no consent packets arrive (`0` keeps idle-safe
  posture; this is the recommended default).
- `gate_motion`: whether consent affects motion outputs.
- `idle_altitude`: normalized hold altitude when gated.
- `idle_jitter`: optional ambient jitter while gated.
- `on_recipe` / `off_recipe`: optional recipe names.
- `auto_recipes`: enables consent-edge recipe switching.

## Consent semantics across subsystems

The same `consent` block is consumed by multiple runtime surfaces. Keep these
semantics aligned when changing defaults or thresholds.

| Subsystem | Config/default source | No-consent-packet behavior | Binary threshold behavior |
| --- | --- | --- | --- |
| Processing tracker HUD | Sketch state + published `/pd/consent` route | Starts/returns to visible OFF state until toggled | Publishes explicit `0`/`1` |
| Control bridge (`osc_msp_bridge.py`) | `consent.default_state` from `config/mapping.yaml` | Mapper initializes to default state; consent `0` keeps neutral MSP heartbeat | Incoming consent is normalized with `>=0.5 => 1` |
| Swarm runtime (`swarm_demo.py`) | `consent.default_state` from loaded mapping/recipe | Starts with default consent and gates behavior via `gate_motion`/`mode` | Startup, OSC handlers, and behavior mapping all normalize consent with `>=0.5 => 1` |
| Operator UI (`operator_ui/state.py`) | UI-local state (defaults to `0`) + API calls | Starts OFF until operator toggles or dispatches consent | API writes normalize consent with `>=0.5 => 1` |
| Stack smoke harness (`scripts/check_stack.py`) | Base mapping/recipe loaded for test run | Harness mapper starts from mapping default and should emit neutral RC frames until consent is armed | Uses bridge consent logic under test (`>=0.5 => 1`) |

Precedence notes:

- OSC consent messages on `osc.address_space.consent` are authoritative for live
  bridge/swarm runs once packets arrive.
- `consent.default_state` is a startup fallback, not a permanent override.
- Operator UI dispatch is an explicit control event (it can drive runtime state
  even when tracker input is idle).

## Consent invariants

Treat these as regression-blocking rules:

- Default startup state is consent OFF (`config/mapping.yaml` keeps `consent.default_state: 0`, operator UI starts at `0`, and starter synthetic tracker defaults to `always_off`).
- Consent normalization is binary and consistent at the control boundary: values `< 0.5` mean OFF, values `>= 0.5` mean ON.
- When consent is OFF, the control bridge must keep sending neutral/safe RC heartbeat frames rather than live gesture-driven RC output.
- Swarm behavior must settle into its configured idle posture when consent is OFF.
- Smoke-harness and UI tests should fail if any subsystem stops honoring these defaults.

## `mapping`

Defines control shaping.

- `mapping.altitude`
  - `deadzone`
  - `curve` (`linear` or `expo`)
  - `gain`
- `mapping.lateral`
  - `deadzone`
  - `curve` (`linear` or `expo`)
  - `gain`
- `mapping.yaw_bias`
  - `bias`
  - `jitter`
- `mapping.leds.palette`
  - color list used by downstream LED systems.
- `mapping.glitch_intensity`
  - `base`
  - `max`

## `bridge`

Defines bridge runtime behavior.

- `hz`: target MSP send rate.
- `mode`: active mode name from `bridge.modes`.
- `stale_after`: seconds of OSC silence before neutralization (if enabled).
- `modes.<name>`
  - optional tuning overrides like `deadzone_boost`, `gain_scale`,
    `jitter_scale`, `aux_strategy`, `neutral_rc`.

## Recipes and overrides

Recipes can override mapping contract values under:

- `control_bridge`

Use `control_bridge.extends` to inherit from a base map (typically
`config/mapping.yaml`) and override only scenario-specific keys.

Reference:

- [`docs/recipes.md`](recipes.md)

## Validation path

Validate mapping and recipe alignment with:

```bash
python3 scripts/validate_config.py
```

## Validation Troubleshooting

| Validator message mentions | What it usually means | Example fix |
| --- | --- | --- |
| `osc.address_space.consent` is missing | The consent OSC route was deleted or misspelled. | Add `consent: "/pd/consent"` under `osc.address_space`. |
| `mapping.<axis>.deadzone` is above the allowed maximum | The deadzone is so large that the axis would effectively stop responding. | Set `mapping.altitude.deadzone: 0.05` or another value below `1`. |
| `mapping.<axis>.curve` is not recognized | The curve name is not one of the supported shaping modes. | Use `curve: "linear"` or `curve: "expo"`. |
| `consent.default_state` is outside the supported range | Startup consent is not a valid OFF/ON-like value. | Set `consent.default_state: 0` for the safe default. |
| `bridge.modes.<name>.aux_strategy` is not recognized | A bridge mode is referencing an unsupported AUX routing strategy. | Use `aux_strategy: full` or `aux_strategy: crowd_only`. |
| `recipe could not be loaded` | The recipe file points at a missing `extends` file or has no valid `control_bridge` block. | Point `control_bridge.extends` at a real file, for example `../mapping.yaml`. |
| `file could not be parsed as YAML/JSON` | The file has a syntax problem such as indentation, a missing colon, or broken brackets. | Recheck spacing and separators; for example `mapping:` on one line and `altitude:` indented below it. |

Use starter preflight for contributor onboarding:

```bash
./scripts/starter_doctor.sh
```
