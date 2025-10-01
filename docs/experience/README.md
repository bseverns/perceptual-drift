# Meta UX Map — Operator vs. Spectator Co-Playbook

This is the bird's-eye choreography for how humans orbit the drone cage. Treat it like a showrunner’s bible: it explains what each role perceives, what they’re responsible for, and how you keep the energy consent-forward. Operator and spectator get equal billing — the piece collapses if either side feels ignored.

## Experience Goals (TL;DR)

- **Operator feels like a dungeon master**, calmly orchestrating chaos with immediate fail-safes.
- **Spectators feel like co-conspirators**, collectively nudging the drone rather than passively watching a screensaver.
- **Shared narrative**: decisions are legible in real-time, and feedback loops are short enough to matter.

## Journey Map — Parallel Tracks

| Phase | Operator POV | Spectator POV | Shared Touchpoints |
| --- | --- | --- | --- |
| **Pre-show** | Run `docs/checklists/safety_checklist.md`, arm the kill switch, stage spare batteries, warm up video feed. | Drift through entry corridor, catch signage, scan QR code linking to safety spiel. | Consent signage glows amber; ambient audio low. |
| **Onboarding** | Launch Processing tracker, verify OSC values in console, simulate MSP override. Narrate rules. | Step on marked floor pads; watch their silhouettes bloom on projection. Receive quick gesture tutorial from facilitator. | Operator toggles “consent” AUX channel → LEDs flip from amber to cyan, signalling interactive mode. |
| **Active play** | Monitor Pi logs + operator monitor. Adjust mapping curves on-the-fly (`config/mapping.yaml`). Maintain situational awareness of drone + crowd. | Experiment with gestures, notice latency (<300 ms target). Hear drone audio + LED colors respond to intensity. Collaborate with other participants to steer vibe. | Shared projection loops FPV feed + motion heatmap. Teensy LEDs mirror OSC confidence. Operator announces mode shifts (“throttle eased, try lateral sways”). |
| **Micro-crisis** | Slam kill switch, announce pause, check battery voltage. Swap prop if needed. | Hear “pause” cue, see LEDs pulse red. Step back to perimeter, wait for restart. | Safety soundtrack fades out & overhead work lights pop on so everyone knows we’re in reset mode. |
| **Cooldown** | Log flight metrics, archive OSC/Mozzi recordings, prep feedback form QR. | Leave a note on wall/QR form, snap selfies with drone (powered down, props off). | Projection loops highlight best crowd motions. Operator shares stats (“you hit 72% crowd sync!”). |

## Sensory Budget

| Channel | Operator Priorities | Spectator Priorities | Safeguards |
| --- | --- | --- | --- |
| Visual | Low-latency monitor, telemetry overlay, LED state machine. | Big projection, cage LEDs telegraph mode, drone glow trails. | Maintain minimum 200 lux house lighting outside cage so operators can read body language. |
| Audio | Intercom headset with FOH, drone motor tone for diagnostics. | Spatialized drone audio + soundtrack reacting to OSC confidence. | Duck music when motors spike >80% throttle to keep OSHA happy. |
| Haptics | Kill switch strap tension, controller haptics (if using gamepad). | Floor pad texture change delineating safe zone. | Anti-fatigue mats reduce slip during excited stomping. |

## Roles & Rituals

- **Operator (1 person minimum)**
  - Rituals: call-and-response safety brief, countdown before arming, audible “landing” cue.
  - Tools: console checklist, laminated quick-recovery flowchart, wrist-mounted kill switch.
  - Success metric: zero unplanned prop contact, smooth pacing between rounds.
- **Spectator-Participants (2–10 inside influence zone)**
  - Rituals: group agreement handshake/fist bump, gesture rehearsal, post-flight debrief question.
  - Tools: floor markings that map gestures to drone axis, HUD overlays showing aggregated inputs.
  - Success metric: high engagement without crowding; participants verbalize understanding of consent gate.
- **Outside Spectators (in lounge area)**
  - Rituals: commentary display summarizing current round, signup slate for next group.
  - Tools: secondary screen with delayed feed + context captions, ambient audio channel.
  - Success metric: people stick around and volunteer to step in, not just doomscroll.

## Failure Modes & Recovery Scripts

| Scenario | Operator Script | Spectator Script | Reset Conditions |
| --- | --- | --- | --- |
| Tracker misreads crowd | “Hold up — I’m recalibrating the tracker. Give me stillness for 10 seconds.” Reboot Processing sketch. | Freeze in place, watch projection fade to grayscale while recalibration happens. | OSC stream stable (<5% jitter) for 30 s, LEDs restore cyan. |
| Drone drifts into net | Kill throttle, announce “net tap — hang tight.” Inspect props, swap battery. | Step back, watch slow-motion replay on projection (optional). | Props intact, battery voltage >3.7 V/cell, net tension ok. |
| Participant overwhelmed | Offer opt-out gesture (raised hand). Fade their contribution from mix via OSC weighting. | Step out gently; floor LED strip guides exit path. | Provide water + bench in decompression zone, log incident in ledger. |

## Instrumentation Checklist

- Log OSC envelopes, drone voltage, and crowd density metrics to correlate human energy with mechanical stress.
- Run a rolling 5-minute heartbeat message on projection summarizing: crowd participation %, drone battery level, current safety status color.
- After each session, append notes to `docs/ASSUMPTION_LEDGER.md` (what surprised us, what hypotheses held, what to tweak).

## Extending the Experience

- **Accessibility**: add alternative control modes (MIDI pads, large buttons) that mirror gestures but respect mobility needs.
- **Remote spectators**: stream FPV feed + gesture overlay with a 30-second delay; include a chat-moderated crowd voting mechanic.
- **Data storytelling**: project heatmaps of crowd input variance between rounds so regulars can see their collaborative fingerprints.

> Bring your own punk rock flourish — zines, patches, neon arrows — but keep the humans feeling safe, seen, and in control of their consent. The art is in the choreography between technician and crowd.
