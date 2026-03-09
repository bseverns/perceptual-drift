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
| Swarm runtime (`swarm_demo.py`) | `consent.default_state` from loaded mapping/recipe | Starts with default consent and gates behavior via `gate_motion`/`mode` | Behavior mapper uses normalized consent (`>=0.5`), OSC edge handlers use `>=1.0` for ON |
| Operator UI (`operator_ui/state.py`) | UI-local state (defaults to `0`) + API calls | Starts OFF until operator toggles or dispatches consent | API writes normalize consent with `>=0.5 => 1` |
| Stack smoke harness (`scripts/check_stack.py`) | Base mapping/recipe loaded for test run | Harness mapper starts from mapping default, then fixture drives updates | Uses bridge consent logic under test |

Precedence notes:

- OSC consent messages on `osc.address_space.consent` are authoritative for live
  bridge/swarm runs once packets arrive.
- `consent.default_state` is a startup fallback, not a permanent override.
- Operator UI dispatch is an explicit control event (it can drive runtime state
  even when tracker input is idle).

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

Use starter preflight for contributor onboarding:

```bash
./scripts/starter_doctor.sh
```
