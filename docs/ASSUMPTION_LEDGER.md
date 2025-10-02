# ASSUMPTION LEDGER

| ID | Assumption | Status | Evidence / Notes | Next Experiment |
| --- | --- | --- | --- | --- |
| **A1** | Indoor micro‑FPV will remain safer than 5″ quads in galleries. | Validated in 2024-03 beta night. | Ran full `docs/checklists/safety_checklist.md` loop and logged zero prop contact in instrumentation notes (see `docs/experience/README.md` sensory budget + recovery scripts). Beta crew reported smoother resets with net + prop guards combo. | Stress-test cage during summer pop-up with doubled crowd density; capture voltage + crowd sync metrics per instrumentation checklist. |
| **A2** | Participants want agency without piloting. | Holding, needs mixed-mobility verification. | Crowd handshake ritual + gesture tutoring from `docs/experience/README.md` kept agency vibes high, but two wheelchair users reported limited influence when floor pads got crowded. Aggregation math in `config/mapping.yaml` still skews toward taller silhouettes. | Prototype elevated input devices (MIDI pads / beam break) and rerun with mobility-diverse group; log perceived agency quotes in session notes. |
| **A3** | Minimal PII (no face storage) is sufficient for interaction fidelity. | Validated against current ethics bar, keep watching. | Privacy stance documented in `docs/PRIVACY_ETHICS.md`; March recordings using silhouette-only pipeline kept OSC latency <300 ms while spectators still tracked their impact. One FOH lead asked for opt-in archival channel — decision deferred. | Draft consent-forward archival flow (opt-in QR) and dry-run with internal crew; measure latency hit vs. baseline silhouette pipeline. |
| **A4** | Analog FPV is acceptable for latency; HD can be added later. | Needs retest with HD feed. | Analog rig kept sub-40 ms latency during 2024-03 beta night, but projection team flagged grain during livestream capture (see `docs/diagrams/system-overview.md` video path). Hardware notes in `hardware/README.md` outline HD upgrade path we haven’t flight-tested. | Integrate HDZero mini bundle with existing VRX capture; bench test against analog for latency + crowd perception before next public showing. |

## Keep This Ledger Loud

This doc only lives if every crew run leaves a breadcrumb. After each session:

1. Drop a dated bullet under the relevant assumption row (use nested lists) noting what held, what cracked, and where the receipts live (logs, clips, witness statements).
2. Link to any fresh config diffs, safety incidents, or debrief recordings so future crews can trace causality without rummaging.
3. If an assumption flips, update the Status column in place and call it out in bold so no one flies on stale lore.

Stay punk, stay accountable — annotate or it didn’t happen.
