# Control Bridge Cheat Sheet

This directory holds the glue that lets weird OSC data pilot a Betaflight quad
through the MSP serial protocol.  The goal is to make the signal path obvious so
you can rip it apart and wire in your own sensors, sequencers, or stage props.

Keep these tabs open while you hack:

* MSP spec crib sheet — https://github.com/betaflight/betaflight.com/blob/master/docs/development/API/MSP-Extensions.md
* python-osc API docs — https://pypi.org/project/python-osc/
* Betaflight RC setup — https://github.com/betaflight/betaflight/wiki/RC-Setup
* Inspiration: [Tello gesture-flight experiments](https://github.com/kinivi/tello-gesture-control),
  [MIT’s Flyfire swarm sketches](https://senseable.mit.edu/flyfire/)

## Pipeline

1. **OSC listeners** soak up altitude/lateral/yaw/crowd/consent values from
   whatever instruments you throw at port `9000`.
2. The `Mapper` class shapes those normalized floats into RC-style microsecond
   values, applying deadzones, expo/linear curves, gains, and trim+jitter bias
   from `config/mapping.yaml`.
3. We slam the values into an MSP `SET_RAW_RC` packet and yeet it down a serial
   line to your flight controller at `115200` baud.  MSP is literally bytes over
   UART — sniff it with `screen /dev/ttyUSB0 115200` to see the `$M<` framing.

## Customizing the mapping

* Edit `config/mapping.yaml` to point new OSC address paths at the keys the
  mapper expects.  You can add more keys—just remember to extend the mapper and
  struct packing.  Example OSC paths from the Processing sketch: `/pd/lat`,
  `/pd/alt`, `/pd/yaw`, `/pd/crowd`, `/pd/consent`.
* Tweak the `deadzone`, `curve`, `expo_strength`, and `gain` fields to match
  your performance rig.  `mapping.altitude` and `mapping.lateral` both support
  `curve: linear` or `curve: expo`.  If you’re new to expo curves, read
  [Joshua Bardwell’s RC expo primer](https://www.youtube.com/watch?v=7zKkRykWq5E).
* `mapping.yaw_bias.bias` trims mechanical drift.  Sprinkle `mapping.yaw_bias.jitter`
  (0.0–0.2 is plenty) when you want deliberate Brownian wobble instead of a
  stick-stiff heading.
* The OSC `crowd` signal now drives AUX outputs: AUX1 mirrors the raw crowd
  energy, AUX2 ramps between `mapping.glitch_intensity.base` ↔ `max`, and AUX3
  picks a color slot from `mapping.leds.palette`.  Plug LEDs, VJ software, or
  synth triggers into those channels downstream.

## Safety checklist

* Keep the `consent` channel low (zero) while experimenting so the bridge keeps
  blasting neutral MSP frames (center sticks, throttle low) at the flight
  controller instead of live commands.  We deliberately keep writing those
  boring packets so Betaflight still sees a heartbeat even though the props stay
  sleepy.
* Use props-off bench tests first.  MSP cares zero percent about your fingers.
* Carry a hardware kill switch if your stage show gets spicy.  BetaFPV LiteRadio
  or Jumper T-Lite transmitters with a ``disarm`` switch are cheap insurance.

## Running the bridge

```bash
python3 osc_msp_bridge.py \
  --serial /dev/ttyUSB0 \
  --baud 115200 \
  --osc_port 9000 \
  --hz 50 \
  --mode smooth
```

If your system needs a different port or baud rate, swap the flags as required.
The bridge now auto-loads `config/mapping.yaml` from the repo root, so skip
`--config` unless you're experimenting with a custom map — in that case point it
wherever your alt mapping lives.
Add `--dry-run` when you want to bench-test without touching the UART — the bridge
will print `[dry-run]` MSP payload stats instead of pushing bytes at the FC.

`--hz` caps the MSP frame rate so you can keep a Pi from firehosing UART. `--mode`
selects one of the `bridge.modes` presets in `config/mapping.yaml`:

* `smooth` — continuous mapping.
* `triggers_only` — fatter deadzones, softer gains, jitter scaled down.
* `idle_visuals` — RC channels park neutral while AUX still reflects the crowd.

Pre-show ritual: run `python3 scripts/validate_config.py` to make sure mapping and
recipes stay sane. If it fails, fix the YAML before you fly.

Now go make something gloriously noisy.  Bonus: use
[`scripts/record_fpv.sh`](../../scripts/record_fpv.sh) to capture your chaos.

## Ghost mode: buffer before you unleash it

Flip on ghost mode with `--ghost-mode` (or `bridge.ghost_mode: true` in your YAML)
to capture a rolling buffer of gesture vectors while consent stays low. The
bridge keeps pushing the neutral RC heartbeat the whole time so Betaflight never
gets lonely. When consent flips high, that pre-roll deque replays in order using
monotonic timestamps, then live OSC data takes over.

Tweak `--ghost-buffer-seconds` (or `bridge.ghost_buffer_seconds`) to set how much
history to stash. Dropping consent back to zero flushes the buffer so the next
arm starts fresh. In `--dry-run` you'll see a HUD suffix like
`ghost:buffering depth=12 window=2.0s` so you know how much is queued.

Need to rehearse end-to-end? `scripts/check_stack.py --ghost-mode` pumps the
fixture frames through the buffer with deterministic timing. Wipe old logs and
artifacts between takes with `scripts/purge_buffers.sh` to keep your captures
clean.
