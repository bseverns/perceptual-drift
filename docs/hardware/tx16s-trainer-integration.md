# TX16S trainer integration reference path

Status: **software boundary UNIT TESTED; hardware UNVERIFIED**.

This is the reference physical architecture for the next Perceptual Drift phase:

```text
Audience / ROI-masked tracker
        ↓ normalized control intent
Intent → SafetyEnvelope → Backend
        ↓ bounded roll/yaw contributions only
USB serial → RP2040/Teensy bridge → wired PPM/CPPM
        ↓
TX16S trainer input → EdgeTX mixer/hard limits → TX16S RF
        ↓
EMAX EZ Pilot Pro
```

The TX16S is always the authoritative human-operated transmitter. Perceptual Drift cannot ARM, change flight mode, or contribute throttle. The initial aircraft profile permits additive roll/yaw influence up to ±0.15, disables pitch, and makes every unsafe or unknown state an explicit zero contribution.

## Evidence vocabulary

- **SOFTWARE VERIFIED**: enforced by code/config validation and covered by automated tests.
- **CONFIG EXPECTED**: declared target configuration, not measured from hardware.
- **OPERATOR VERIFIED**: a human explicitly checked the physical configuration for the current session.
- **UNVERIFIED**: not measured or confirmed.

Longer maturity labels are `ASPIRATIONAL`, `SIMULATED`, `UNIT TESTED`, `BENCH TESTED`, `HIL VERIFIED`, `FLIGHT TESTED`, and `PUBLICLY REHEARSED`. Do not promote a claim without retaining the corresponding evidence.

## Software contract

`software.trainer_control.intent.ControlIntent` contains only normalized roll, pitch, yaw, throttle, crowd, consent, and timestamp values. It knows nothing about MSP, PPM, SBUS, RC microseconds, receiver protocols, or ARM.

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

## Expected TX16S setup—not software-verified

The declared target in `config/transmitters/tx16s_pd.yaml` is:

- master transmitter role;
- trainer roll `ADD`, weight no more than 15%;
- trainer yaw `ADD`, weight no more than 15%;
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
