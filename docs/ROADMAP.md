# Perceptual Drift Implementation Roadmap (2026 Q2)

This file tracks execution against the current improvement plan. It is written
for operators and contributors who need concrete scope, sequence, and exit
criteria.

## Milestones

| Milestone | Window | Status | Primary outcome |
| --- | --- | --- | --- |
| M0: Baseline + scope lock | Week 1 | completed | Architecture baseline, dependency matrix, starter scope |
| M1: Starter bundle | Weeks 2-3 | in progress | One-command local launch for tracker + bridge (+ optional video) |
| M2: Environment reproducibility | Weeks 4-6 | in progress | Pinned dependency flow for desktop and Jetson |
| M3: Docs consolidation | Weeks 7-8 | completed | Canonical mapping contract and reduced doc duplication |
| M4: Swarm hardening | Weeks 9-10 | completed | Latency and collision safety checks with replayable tests |
| M5: Operator UI alpha | Weeks 11-12 | in progress | Non-CLI control surface for recipes, consent, and curves |

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

## M1 Deliverables (starter bundle)

- [x] Add `scripts/starter_doctor.sh` preflight checks.
- [x] Add `scripts/starter_up.sh` orchestration launcher.
- [x] Add minimal tracker with `synthetic` and `camera` modes.
- [ ] Validate first-run on one desktop and one Jetson image.
- [ ] Document first-run path to <= 30 minutes in rehearsal notes.

### M1 exit criteria

- Fresh clone can run:
  - `./scripts/starter_doctor.sh`
  - `./scripts/starter_up.sh`
- Startup failures are actionable (clear error messages, no silent hangs).

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
- [ ] Add session export: active config + telemetry snapshot.

### M5 exit criteria

- Non-technical operator can run a rehearsal without editing YAML.
