# Control Bridge Cheat Sheet

This directory holds the glue that lets weird OSC data pilot a Betaflight quad
through the MSP serial protocol.  The goal is to make the signal path obvious so
you can rip it apart and wire in your own sensors, sequencers, or stage props.

Keep these tabs open while you hack:

* MSP spec crib sheet — https://github.com/betaflight/betaflight/wiki/MSP
* python-osc API docs — https://pypi.org/project/python-osc/
* Betaflight RC setup — https://github.com/betaflight/betaflight/wiki/RC-Setup
* Inspiration: [DJI FPVgestures](https://github.com/whoisandrewd/fpv-gestures),
  [MIT Swarm Lab](https://aerobotics.mit.edu/)

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

* Keep the `consent` channel low (zero) while experimenting so packets don't hit
  the flight controller.
* Use props-off bench tests first.  MSP cares zero percent about your fingers.
* Carry a hardware kill switch if your stage show gets spicy.  BetaFPV LiteRadio
  or Jumper T-Lite transmitters with a ``disarm`` switch are cheap insurance.

## Running the bridge

```bash
python3 osc_msp_bridge.py \
  --serial /dev/ttyUSB0 \
  --baud 115200 \
  --config ../../config/mapping.yaml \
  --osc_port 9000
```

If your system needs a different port or baud rate, swap the flags as required.

Now go make something gloriously noisy.  Bonus: use
[`scripts/record_fpv.sh`](../../scripts/record_fpv.sh) to capture your chaos.
