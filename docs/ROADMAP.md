# Perceptual Drift Implementation Roadmap (2026 Q2)

This file tracks execution against the current improvement plan. It is written
for operators and contributors who need concrete scope, sequence, and exit
criteria.

## Milestones

| Milestone | Window | Status | Primary outcome |
| --- | --- | --- | --- |
| M0: Baseline + scope lock | Week 1 | completed | Architecture baseline, dependency matrix, starter scope |
| M1: Safe rehearsal path + starter bundle | Weeks 2-3 | in progress | Canonical no-hardware safe path plus lower-level manual launcher |
| M2: Environment reproducibility | Weeks 4-6 | in progress | Pinned dependency flow for desktop and Jetson |
| M3: Docs consolidation | Weeks 7-8 | completed | Canonical mapping contract and reduced doc duplication |
| M4: Swarm hardening | Weeks 9-10 | completed | Latency and collision safety checks with replayable tests |
| M5: Operator UI alpha | Weeks 11-12 | in progress | Non-CLI control surface for recipes, consent, and curves |
| M6: TX16S trainer boundary | Physical integration phase | in progress | One master TX16S, one aircraft, staged props-off trainer path |

M6 explicitly supersedes swarm expansion as the active physical priority. Software boundary tests do not advance the hardware evidence stage.

## M0 Deliverables (baseline + scope lock)

- [x] Create roadmap with milestones and acceptance criteria.
- [x] Create dependency matrix with pinned versions and known friction points.
- [x] Define starter bundle scope and "what is out of scope" for v0.
- [ ] Align support matrix with field rigs after 2 rehearsal sessions.

### M0 scope decisions

- Starter bundle v0 includes:
  - Minimal tracker (`software/starter-bundle/minimal_tracker.py`)
  - OSC->MSP bridge (`software/control-bridge/osc_msp_bridge.py`)
  - Optional GStreamer preview (`software/video-pipeline/gst_launch.sh`)
- Starter bundle v0 intentionally excludes:
  - Processing IDE dependency
  - ROS2/CrazySwarm2
  - Teensy firmware flashing flows

## M1 Deliverables (safe rehearsal path + starter bundle)

- [x] Add `scripts/starter_doctor.sh` preflight checks.
- [x] Add `scripts/starter_up.sh` orchestration launcher.
- [x] Add minimal tracker with `synthetic` and `camera` modes.
- [x] Add `pd-safe-rehearsal` as the canonical no-hardware first command.
- [x] Add `pd-status` for local observability of the rehearsal bundle.
- [x] Document consent OFF as the default safe-path invariant.
- [ ] Validate software-first run path on one desktop and one Jetson image.
- [ ] Validate field rehearsal with operators on physical rig.
- [ ] Document first-run path to <= 30 minutes in rehearsal notes.

### M1 exit criteria

Software safe-path readiness:

- Fresh clone can run:
  - `pd-safe-rehearsal`
  - `pd-status`
  - `pd-safe-rehearsal stop`
- Consent stays OFF by default on no-hardware startup.
- Startup failures are actionable (clear error messages, no silent hangs).

Lower-level manual path remains available:

- `./scripts/starter_doctor.sh`
- `./scripts/starter_up.sh`

Field validation remains separate from code proof:

- One desktop and one Jetson image complete the no-hardware flow.
- Physical rehearsal still requires rig-specific verification, spotter signoff,
  and hardware-specific safety checks.

## M2 Deliverables (reproducible dependencies)

- [x] Add `requirements-starter.txt` with pinned Python dependencies.
- [x] Add lock/constraints strategy for bridge + starter scripts.
- [x] Add container/dev environment definition matching CI dependency floor.
- [x] Add Jetson-specific reproducible setup profile with explicit versions.
- [ ] Validate profile on one fresh Jetson flash and one clean Linux container.

### M2 exit criteria

- No manual dependency "guessing" for starter bundle on supported platforms.
- CI and local starter command paths share the same dependency pins.

## M3 Deliverables (docs consolidation)

- [x] Create canonical mapping reference doc.
- [x] Replace repeated mapping descriptions with links from:
  - `docs/control-stack-playbook.md`
  - `docs/recipes.md`
- [x] Add concise doc IA index page ("start", "operate", "extend", "swarm").

### M3 exit criteria

- One source of truth per concept.
- Duplicate mapping sections removed or reduced to short pointers.

## M4 Deliverables (swarm hardening)

- [x] Add swarm latency benchmark harness with p50/p95 capture.
- [x] Add collision envelope checks in bridge/sim loop.
- [x] Add multi-user gesture replay scenarios for swarm mode.

### M4 exit criteria

- Published swarm limits and tested latency budget.
- Repeatable safety and behavior checks in simulation.

## M5 Deliverables (operator UI alpha)

- [x] Build web UI alpha for:
  - Recipe load/switch
  - Consent state monitor
  - Mapping curve visualization
- [x] Add session export: active config + telemetry snapshot.
- [x] Add runtime health visibility for bridge/tracker/sim daemons.
- [x] Add starter runtime start/stop controls in operator UI.
- [x] Add guided rehearsal flow (preflight + profile + named session).

### M5 exit criteria

- Non-technical operator can run a rehearsal without editing YAML.

## M6 Deliverables (TX16S trainer boundary)

Software boundary:

- [x] Define normalized `ControlIntent`, immutable aircraft limits, and a
  fail-closed `SafetyEnvelope`.
- [x] Add a real serial trainer runtime with independent OSC/consent freshness,
  MCU heartbeat gating, declared aircraft/TX16S profile validation, and neutral
  shutdown.
- [x] Place transport-neutral artistic/recipe mapping before the safety envelope.
- [x] Log effective trainer state transitions to the operational JSONL ledger.
- [x] Cover active output plus independent stale-input, consent, heartbeat, and
  operator-disable transitions in automated tests.
- [x] Add a read-only preflight mode that observes MCU heartbeats without sending
  trainer commands.

Physical evidence progression:

- [x] Build the RP2040/Pico trainer-bridge target in a clean PlatformIO flow.
- [ ] Flash the selected trainer bridge board.
- [ ] Inspect PPM timing, polarity, voltage, channel order, and watchdog behavior
  on a scope or logic analyzer with no transmitter attached.
- [ ] Connect the bridge to TX16S with no aircraft and verify trainer ADD/OFF
  modes, the physical enable switch, and the expected maximum 2.25% channel
  contribution in the EdgeTX monitor.
- [ ] Connect the EMAX with props removed and verify manual authority, zero
  software throttle/pitch/ARM, and zero contribution for every host/bridge loss.
- [ ] Complete contained manual flight before introducing bounded synthetic or
  participant influence.

### M6 exit criteria

- Software evidence remains labeled `UNIT TESTED` until physical measurements
  are retained.
- Scope/analyzer evidence establishes the bridge signal and watchdog behavior.
- EdgeTX monitor evidence establishes actual final channel authority and the
  pilot's physical override path.
- Props-off aircraft evidence confirms all software loss conditions produce zero
  trainer contribution without impairing pilot control.
