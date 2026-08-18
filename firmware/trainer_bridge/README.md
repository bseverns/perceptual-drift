# Wired trainer bridge scaffold

Status: **UNIT TESTED at the host protocol boundary; firmware build and electrical output are UNVERIFIED**.

This scaffold targets an RP2040/Pico by default and also declares a Teensy 4.0 PlatformIO environment. It accepts bounded, inspectable USB-serial packets from Perceptual Drift and produces a conventional centered PPM/CPPM channel stream for a future TX16S trainer connection.

It does not transmit RF, configure EdgeTX, ARM, select flight modes, or produce an active throttle contribution.

## Host protocol

At 115200 baud, the host sends ASCII lines:

```text
PD1,<sequence>,<roll>,<pitch>,<yaw>,0.000000,<xor-checksum-hex>\n
```

The checksum is an eight-bit XOR of every ASCII byte before the final comma. Axes are normalized contributions in `[-1, 1]`, not RC microseconds. Firmware independently clamps roll/yaw to ±0.15 in this trainer domain and forces pitch/throttle to zero. The declared EdgeTX 15% trainer weight is a separate downstream multiplier, so the expected final contribution is at most 2.25% of full stick travel (`0.15 × 0.15`); that result remains hardware-unverified until checked in the EdgeTX channel monitor.

The bridge emits independently generated heartbeats:

```text
HB,<sequence>\n
```

If a packet is malformed/non-finite/out of range, or valid packets stop for 250 ms, every contribution becomes centered zero. The last command is never held indefinitely.

## Build only

```bash
pio run -d firmware/trainer_bridge -e pico
pio run -d firmware/trainer_bridge -e teensy40
```

The matching installed host process is `pd-trainer-run`. With the bridge
attached over USB, start neutral-only discovery with:

```bash
pd-trainer-run --serial /dev/ttyACM0
```

See `docs/hardware/tx16s-trainer-integration.md` before adding the explicit
`--operator-enable` software gate.

Do not flash or connect this scaffold to a TX16S until the selected board, signal voltage, trainer-jack pinout, required PPM polarity, channel order, and common-ground arrangement have been measured or confirmed from authoritative hardware documentation. First inspect PPM on a scope or logic analyzer with no transmitter or aircraft attached.
