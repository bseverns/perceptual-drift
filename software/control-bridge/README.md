# Control Bridge Cheat Sheet

This directory holds the glue that lets weird OSC data pilot a Betaflight quad
through the MSP serial protocol.  The goal is to make the signal path obvious so
you can rip it apart and wire in your own sensors, sequencers, or stage props.

## Pipeline

1. **OSC listeners** soak up altitude/lateral/yaw/crowd/consent values from
   whatever instruments you throw at port `9000`.
2. The `Mapper` class shapes those normalized floats into RC-style microsecond
   values, applying expo curves, gains, and trim bias from `config/mapping.yaml`.
3. We slam the values into an MSP `SET_RAW_RC` packet and yeet it down a serial
   line to your flight controller at `115200` baud.

## Customizing the mapping

* Edit `config/mapping.yaml` to point new OSC address paths at the keys the
  mapper expects.  You can add more keys—just remember to extend the mapper and
  struct packing.
* Tweak the `gain`, `curve`, and `bias` fields to match your performance rig.
  The comments in `osc_msp_bridge.py` call out each knob.

## Safety checklist

* Keep the `consent` channel low (zero) while experimenting so packets don't hit
  the flight controller.
* Use props-off bench tests first.  MSP cares zero percent about your fingers.
* Carry a hardware kill switch if your stage show gets spicy.

## Running the bridge

```bash
python3 osc_msp_bridge.py \
  --serial /dev/ttyUSB0 \
  --baud 115200 \
  --config ../../config/mapping.yaml \
  --osc_port 9000
```

If your system needs a different port or baud rate, swap the flags as required.

Now go make something gloriously noisy.
