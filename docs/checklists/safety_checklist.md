# Safety Checklist & Run Log

> Print it, laminate it, and mark it up like you mean it. Every step is both a ritual and documentation. Link out as needed: [Flight Cage Build Notes](../../hardware/flight_cage/README.md) · [Control Bridge Playbook](../control-stack-playbook.md) · [Experience Choreography](../experience/README.md).

## Sectioned Checklist

Each block is a call-and-response between **Lead Operator (LO)** and **Safety Second (S2)**. The third voice is the log — initial each line so the paper trail sticks.

### 1. Preflight — before anyone is near the net

| Step | Call → Response | Tolerance / Target | Why we care | If this flunks | LO | S2 |
| --- | --- | --- | --- | --- | --- | --- |
| Cage integrity | LO: “Net sweep complete?” → S2: “Net is taut, no rips, tension straps locked.” | Net deflection ≤ 5 cm when pressed mid-span. | Keeps carbon + flesh separated. [Flight cage spec](../../hardware/flight_cage/README.md#netting--tension) backs it up. | Patch or zip-tie tear before proceeding. If structural, abort and relocate crowd. | | |
| Consent gate idle | LO: “Consent gate status?” → S2: “Gate LED amber, AUX channel low, queue is staged.” | AUX channel reads <1300 µs; signage lit amber. | Matches the human choreography in [experience README](../experience/README.md#journey-map--parallel-tracks). | Reboot consent microcontroller; if LEDs stay wrong, hold briefing until resolved. | | |
| Kill-switch drill | LO: “Kill-switch muscle memory?” → S2: taps kill-switch twice, replies “Kill path clear.” | Kill switch cuts power within 1 s; strap secured to wrist/belt. | Practicing the slam before chaos is required in the experience flow. | Replace batteries or wiring if latency >1 s; run second drill. Abort session if not crisp. | | |
| Power chain | LO: “Battery voltage?” → S2: “Pack resting at ≥ 16.8 V (4S) or ≥ 12.6 V (3S).” | ≥ 4.2 V/cell resting; reject <4.1 V/cell for first sortie. | Avoids sag mid-crowd, keeps ESC temps sane. | Swap pack, log as “REJECTED” with voltage; stash in LiPo bag. | | |
| Fire tools | LO: “Extinguisher + LiPo bag?” → S2: “Extinguisher pinned, gauge in green. LiPo bag zipped, 1 m from cage.” | ABC extinguisher pressure in green zone. | If we set something on fire, we put it out. | Replace or refill extinguisher; move LiPo bag into reach before continuing. | | |
| Control bridge sync | LO: “OSC→MSP bridge alive?” → S2: “Bridge streaming, RC spans 1100–1900 µs.” | Oscilloscope/Betaflight Receiver tab shows jitter < ±5 µs. | Guarantees gesture data won’t brick the FC. [Control-stack playbook](../control-stack-playbook.md#osc-to-msp-bridge) covers tuning. | Restart bridge script; if still noisy, fall back to manual radio and note in log. | | |
| Space hygiene | LO: “Floor and cables?” → S2: “No FOD, cables taped or rerouted.” | No loose gear within 1 m of cage perimeter. | Trip hazards are not punk, they’re lawsuits. | Sweep again; reroute or tape cables before audience entry. | | |

### 2. Just Before Arming — after briefing, before props spin

| Step | Call → Response | Tolerance / Target | Why we care | If this flunks | LO | S2 |
| --- | --- | --- | --- | --- | --- | --- |
| Audience placement | LO: “Perimeter clear?” → S2: “Participants toes behind tape, observers back 1 m.” | Minimum 1 m standoff from net; marked pads occupied. | Prevents reach-ins when adrenaline spikes. | Re-brief crowd; pause until compliance. Note any friction in log. | | |
| Consent gate open | LO: “Ready to flip consent?” → S2: “Gate armed — LEDs cyan, crowd acknowledged.” | AUX channel >1700 µs; LED swap verified. | Reinforces the consent choreography in [experience doc](../experience/README.md#roles--rituals). | If LEDs fail, revert to amber and troubleshoot. If crowd unclear, repeat call-and-response. | | |
| Sensor sanity | LO: “Tracking feed clean?” → S2: “Yes — OSC jitter <5%, frame rate ≥ 30 fps.” | Check console metrics; latency target <300 ms. | Keeps gesture-to-drone feedback playable. | Reboot tracker, reduce resolution, or switch to manual mode. If unresolved, abort interactive mode. | | |
| FC arming posture | LO: “Flight controller mode?” → S2: “Angle mode, throttle cap 60%, failsafe tested.” | Betaflight OSD shows ANGLE; throttle limit ≤ 0.6; failsafe triggers disarm within 1 s. | Avoids runaway throttle and holds us inside choreographed envelope. | Reflash profile or reload CLI dump; if failsafe misbehaves, do not arm. | | |
| Dry-run hover | LO: “Ghost run?” → S2: “Dry-run complete — no audience, 30 s stable hover logged.” | Voltage sag <0.5 V/cell during hover; temps normal. | Confirms rig before inviting humans into loop. | Diagnose vibe: check props, recalibrate gyro, or swap drone. Restart checklist after fix. | | |

### 3. Post-Run — immediately after disarm

| Step | Call → Response | Tolerance / Target | Why we care | If this flunks | LO | S2 |
| --- | --- | --- | --- | --- | --- | --- |
| Disarm confirmation | LO: “Motors cold?” → S2: “Props static, LiPo disconnected, kill switch still within reach.” | Motor temp <40 °C by touch; pack unplugged within 15 s. | Physical reset before crowd swarms the rig. | If motors hot, add cool-down fan; log and delay next round. | | |
| Consent gate reset | LO: “Gate back to amber?” → S2: “Consent closed, LEDs amber, signage re-lit.” | AUX channel <1300 µs; signage lighting matches idle. | Signals safe-state to next group; keeps ritual intact. | If LEDs stick, power-cycle gate controller; hold audience until fixed. | | |
| Battery log | LO: “Voltage after flight?” → S2: “Post-run at ≥ 3.7 V/cell.” | Resting ≥ 14.8 V (4S) / 11.1 V (3S). | Avoids over-discharging; informs charge rotation. | If below threshold, mark pack for gentle charge and inspect for puffing. | | |
| Debrief pulse | LO: “Crew notes?” → S2: “Latency, crowd vibe, anomalies logged.” | Complete run log within 5 min; include consent/killswitch notes. | Documentation fuels iterative safety. | If skipped, run mini-retro before next session; assign scribe. | | |
| Reset space | LO: “Floor staged?” → S2: “Cables back, signage visible, decompression zone stocked.” | House lights ≥200 lux; water + bench restocked. | Sets tone for next round, supports decompression ritual. | If supplies low, restock before announcing next group. | | |

## Contingency Flow (Quick Reference)

- **Consent gate failure mid-run**: Slam kill switch, announce “Consent pause,” flip LEDs to red, guide participants out via floor strip. Resume only after gate hardware passes dry-run.
- **Kill switch latency >1 s during checks**: Replace cable or switch body, re-run drill, and note in log. Do not advance checklist until response <1 s.
- **Audience breaches perimeter**: LO calls “Reset line,” S2 escorts crowd back. Repeat boundary cue from experience README; restart section 2.
- **Telemetry noise >±5 µs**: Drop to manual RC control, capture OSC logs, and schedule bridge debug per control-stack playbook.

## Run Log Template

Fill this every session. File copies in `docs/logs/` or scan to archive.

| Date | Session ID | Drone ID | Battery (pre/post V) | Consent Gate Status Notes | Kill-Switch Drill Outcome | Anomalies / Contingencies Triggered | Crowd Feedback Highlights | LO Initials | S2 Initials |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |

Add extra rows as needed, and staple any incident reports or photos behind this sheet. Punk rock is cool; documented punk rock is safer.
