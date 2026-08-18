# TX16S trainer integration reference path

Status: **software boundary UNIT TESTED; hardware UNVERIFIED**.

This is the reference physical architecture for the next Perceptual Drift phase:

```text
Audience / ROI-masked tracker
        ↓ normalized tracker signals
artistic / recipe IntentMapper → ControlIntent
        ↓
SafetyEnvelope → Backend
        ↓ bounded roll/yaw contributions only
USB serial → RP2040/Teensy bridge → wired PPM/CPPM
        ↓
TX16S trainer input → EdgeTX mixer/hard limits → TX16S RF
        ↓
EMAX EZ Pilot Pro
```

The TX16S is always the authoritative human-operated transmitter. Perceptual Drift cannot ARM, change flight mode, or contribute throttle. The initial aircraft profile permits additive roll/yaw values up to ±0.15 **in the normalized trainer packet domain**, disables pitch, and makes every unsafe or unknown state an explicit zero contribution.

### What the two 15% limits mean

The software profile and trainer-bridge firmware both cap the normalized packet/PPM contribution at ±0.15. Those are redundant checks on the same trainer-domain value; they do not multiply each other. The declared EdgeTX configuration then applies a separate 15% trainer weight when adding that input to the pilot's stick. Under ordinary EdgeTX normalized mixer semantics, the current expected maximum algorithmic contribution at the final TX stick is therefore `0.15 × 0.15 = 0.0225`, or 2.25% of full stick travel—not 15%.

That 2.25% figure is **CONFIG EXPECTED**, not hardware verified. Confirm it in the EdgeTX channel monitor and on the trainer signal during bench stage 3. Do not describe the installation as having “15% final authority”; use “±15% trainer-domain input, additionally weighted to 15% by EdgeTX” until measurements establish the actual final channel contribution.

## Evidence vocabulary

- **SOFTWARE VERIFIED**: enforced by code/config validation and covered by automated tests.
- **CONFIG EXPECTED**: declared target configuration, not measured from hardware.
- **OPERATOR VERIFIED**: a human explicitly checked the physical configuration for the current session.
- **UNVERIFIED**: not measured or confirmed.

Longer maturity labels are `ASPIRATIONAL`, `SIMULATED`, `UNIT TESTED`, `BENCH TESTED`, `HIL VERIFIED`, `FLIGHT TESTED`, and `PUBLICLY REHEARSED`. Do not promote a claim without retaining the corresponding evidence.

## Software contract

`software.trainer_control.intent.ControlIntent` contains only normalized roll, pitch, yaw, throttle, crowd, consent, and timestamp values. It knows nothing about MSP, PPM, SBUS, RC microseconds, receiver protocols, or ARM.

`software.trainer_control.mapping.IntentMapper` is the transport-neutral artistic stage. It applies the canonical lateral deadzone, curve, and gain plus yaw bias/jitter from `config/mapping.yaml` or a validated recipe. It produces normalized intent and has no knowledge of aircraft authority or serial encoding.

`SafetyEnvelope` loads `config/aircraft/ez_pilot_pro.yaml`. It bypasses artistic mapping and produces zero axes on startup, consent OFF, stale/malformed/non-finite input, operator disable, missing/stale bridge heartbeat, or invalid safety profile. Valid inputs are hard-clamped after artistic shaping, so recipes cannot raise physical authority.

Backends are downstream of this boundary:

- `DryRunBackend`: logs complete effective state; opens no device.
- `TrainerBackend`: emits the inspectable wired bridge protocol; always encodes zero throttle.
- `LegacyMSPBackend`: retains an experimental direct-MSP adapter without ARM or mapped throttle.

The existing `osc_msp_bridge.py` remains available for software rehearsal and historical experiments. It is not the reference real-aircraft path.

## Clean setup and software-only rehearsal

Python 3.10 matches CI; the project supports Python 3.10 or newer.

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r requirements-dev.txt -c constraints/py310-linux.txt
python -m pip install -e . --no-build-isolation
python -m pytest tests -q
pd-validate-config
pd-safe-rehearsal
```

In another shell, inspect with `pd-status`, then stop with `pd-safe-rehearsal stop`.

Trainer-path commands:

```bash
./scripts/starter_doctor.sh --trainer
pd-trainer-preflight
pd-trainer-dry-run
```

The default dry run deliberately reports `SAFE — ZERO ALGORITHMIC AUTHORITY`: consent, operator enable, and a real bridge heartbeat are absent. To exercise only the bounded software calculation, with no device opened:

```bash
pd-trainer-dry-run --consent --operator-enable \
  --simulate-bridge-heartbeat --roll 1 --pitch 1 --yaw -1
```

That simulation must report roll `0.15`, pitch `0`, yaw `-0.15`, and throttle `0`. A simulated heartbeat is labeled simulated and is never evidence of a connected bridge.

## Live host runtime

`pd-trainer-run` is the installed reference host process. It joins the canonical OSC routes, artistic mapper, `SafetyEnvelope`, the real USB-serial `TrainerBackend`, and the bridge's independently generated `HB,<sequence>` heartbeat. It validates both the aircraft and declared TX16S profiles before opening the live path. It listens on loopback by default and transmits neutral packets unless every required condition is current: lateral input, yaw input, consent heartbeat, bridge heartbeat, valid profiles, and explicit local `--operator-enable`.

For a props-off bench session:

```bash
pd-trainer-run --serial /dev/ttyACM0
```

That command is intentionally neutral-only. After confirming the serial device, heartbeat, PPM output, TX monitor, and physical trainer-enable switch, restart with the local software gate:

```bash
pd-trainer-run --serial /dev/ttyACM0 --operator-enable
```

The live trainer runtime maps canonical `/pd/lat` through lateral deadzone/curve/gain to trainer roll and maps `/pd/yaw` through configured bias/jitter to trainer yaw. Select an initial recipe with `--recipe ambient`; validated `/pd/patch` string messages can change recipes while running. `/pd/alt` is deliberately not connected because software pitch and throttle are forbidden. It consumes `/pd/crowd` for telemetry and `/pd/consent` as a required heartbeat. Invalid/non-finite OSC values do not refresh input freshness. A partial OSC stream cannot keep an old roll, yaw, or consent value fresh because those timestamps expire independently.

The console reports effective state transitions—not every frame—and writes the same events to `logs/ops_events.jsonl`, for example `TRAINER STATE → ACTIVE`, `TRAINER STATE → NEUTRAL: stale_input`, and `TRAINER STATE → NEUTRAL: trainer_bridge_heartbeat_lost`. The Operator UI runtime-health view can discover `pd-trainer-run`; it does not start or enable the physical trainer path.

For a read-only Stage 1→2 serial check, request several MCU heartbeats without transmitting trainer commands:

```bash
pd-trainer-preflight --serial /dev/ttyACM0
```

Success is labeled `SOFTWARE OBSERVED: trainer bridge responding`; the PPM electrical signal remains `HARDWARE SIGNAL: still UNVERIFIED` until measured.

Do not expose the OSC socket on a venue LAN. A non-loopback `--bind` is an explicit isolated-rig override and prints a warning. The physical TX16S trainer switch remains the final authority even when `--operator-enable` is present.

## Expected TX16S setup—not software-verified

The declared target in `config/transmitters/tx16s_pd.yaml` is:

- master transmitter role;
- trainer roll `ADD`, weight no more than 15% of the already bounded trainer-domain input;
- trainer yaw `ADD`, weight no more than 15% of the already bounded trainer-domain input;
- trainer pitch and throttle `OFF`;
- ARM and flight mode on physical transmitter switches only;
- Perceptual Drift authority behind a dedicated physical trainer-enable switch.

The operator confirmed on 2026-08-12 that this TX16S supports FrSky D8 and that the RF function operates correctly. Binding, range, failsafe, and channel behavior with the actual EZ Pilot Pro remain unverified. Confirm EdgeTX version, trainer jack pinout/electrical levels, PPM polarity/frame timing, channel order, mixer behavior, and failsafe using the real transmitter. `pd-trainer-preflight --operator-confirm-tx16s` records no durable proof; it only labels the current console readout after the operator has actually checked the radio.

## Bench progression

Do not collapse these gates:

| Stage | Scope | Exit evidence |
| --- | --- | --- |
| 0 | Existing software-only safe rehearsal | consent starts OFF; tests/smoke pass |
| 1 | Trainer backend dry-run; no transmitter | limits and every neutral reason observed |
| 2 | Microcontroller output; no aircraft/transmitter | PPM inspected on scope/analyzer; watchdog returns all channels to center |
| 3 | TX16S trainer input; no armed aircraft | EdgeTX monitor confirms add/off modes and physical enable/disable |
| 4 | Aircraft connected, props removed | manual control; bounded roll/yaw; zero pitch/throttle; no software ARM; host/bridge failures zero contribution |
| 5 | Manual contained flight only | stock aircraft characterized and logs retained |
| 6 | Pilot plus small synthetic roll/yaw | bounded influence demonstrated under pilot authority |
| 7 | One consenting participant | ROI, opt-out, and operator response verified |
| 8 | Group behavior | only after prior evidence is reviewed |

Props remain off through Stage 4. Stage 5 is not authorized merely because software tests pass.

## Physical blockers

Before props can be installed, complete the arrival checklist, archive the factory configuration, bind and characterize the stock aircraft, verify the physical TX16S authority/enable/failsafe paths, validate bridge electrical/PPM behavior and watchdog on instruments, test host and bridge power-loss behavior props-off, and use an appropriate contained-flight risk assessment. No code result substitutes for these checks.
