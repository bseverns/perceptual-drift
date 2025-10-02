# LED Wiring (WS2812)

Welcome to the glow-stick heart of Perceptual Drift. This page shows you how to
keep a Teensy, a resistor, a capacitor, and a WS2812 strip from tearing each
other apart mid-show. Treat it like a lab notebook crossed with a zine.

## Top-down wiring map

```
                   Top-down harness (not to scale)

     Teensy 4.x header (VIN side toward USB)
     ┌──────────────────────────────────────────────────────────────┐
     │ [GND] o───┐                                                  │
     │           │             1000 µF electrolytic                 │
     │ [VIN] o───┼───────┐      +││−   strap the can within 2 cm    │
     │           │       │      ││                                 │
     │ [PIN 6] o─┴─[330 Ω]─▶ DATA ───────▶ DIN  WS2812 strip  ▶▶▶   │
     │                           │                                  │
     └───────────────────────────┴──────────────┬───────────────────┘
                                                 │
                               5 V rail ─────────┘────▶ 5 V pad on strip
                               GND rail ─────────────────▶ GND pad on strip
```

* **Connector orientations**: the Teensy silkscreen text reads upright when the
  USB port faces left. VIN is the second pin from the corner; ground sits right
  beside it. Route the data resistor inline as close to the Teensy pin as you
  can so the strip only sees a single stiff trace. Electrolytic capacitor
  polarity matters — the marked (negative) lead jumps onto the ground bus.
* **Strip arrow gospel**: WS2812 arrows must point *away* from the Teensy. If
  you mount the strip backwards the data simply dies. Verify the arrow before
  you heatshrink anything.
* **Strain relief**: lash the strip lead bundle to the airframe or console with
  fabric tape or a zip-tie loop before it hits the solder joints. WS2812 pads
  rip off faster than you can say “where’d the red channel go?”

## Recommended wiring specs

* **Power and ground**: 20–22 AWG silicone wire up to 1.5 m runs. If you must
  go further, bump to 18 AWG or split power injection at both ends of the strip.
* **Data**: 24–26 AWG is plenty. Shielded microphone cable makes it happy in
  RF-ugly venues.
* **Connectors**: JST-SM 3-pin works for modular rigs; lock it with hot glue so
  the tab never pops mid-flight. For bench rigs, Dupont jumpers are fine but
  expect to re-crimp often.

## Bring-up checklist (a ritual, not optional)

1. **Continuity + polarity**
   * Kill power. Multimeter beep every 5 V → 5 V and GND → GND path.
   * Confirm there is *no* short between 5 V and GND. A 1000 µF cap will hide a
     brief blip; wait for the reading to settle.
2. **Load the rainbow sketch**
   * Flash `firmware/teensy-led/src/main.cpp` straight onto the Teensy using
     Arduino IDE or `arduino-cli upload`. The default loop spits rainbows and
     listens for serial `C/I` commands.
   * Power the strip from a lab supply at 5.0 V. Watch for the initial blue
     glow — that’s the firmware confirming it’s alive.
3. **Current sanity check**
   * Drive a `C 255 255 255` followed by `I 255` over USB serial. Expect ~60 mA
     per pixel (0.72 A for a 12 px strip). If you’re wildly above that, your
     supply is dirty or the strip is shorting segments.
   * Back the intensity down (`I 64`) and confirm the current scales linearly.
4. **Cable wiggle test**
   * Tug on every connector and strain relief while the strip runs the rainbow.
     Any flicker means redo the joint now, not in front of an audience.

## Troubleshooting quick-and-dirty matrix

| Symptom                 | Likely cause(s)                                      | Fix it like you mean it |
| ----------------------- | ---------------------------------------------------- | ----------------------- |
| Random flicker          | Data line picked up RF, or ground reference floating | Shorten data lead, twist data+ground, add 100 Ω in series if the run is long |
| Brownouts / resets      | Cap missing, supply sagging, cable too thin          | Install the 1000 µF cap, upgrade to 3 A regulator, use thicker power wire |
| Data corruption / color | Resistor missing, strip backwards, noise on VIN      | Verify 330 Ω resistor is inline, check arrow direction, scope VIN for dips |
| Whole strip dark        | Reversed power, dead first LED, no common ground     | Recheck polarity before powering, inject 5 V downstream, tie Teensy GND to strip |

## Control story: from OSC crowd to serial `C/I`

1. **Crowd energy arrives over OSC** as `/pd/crowd` (0.0–1.0 floats) from the
   Processing tracker rig documented in `config/mapping.yaml`.
2. The **OSC→MSP bridge** (`software/control-bridge/osc_msp_bridge.py`) already
   quantizes that crowd value into auxiliary channels: AUX1 mirrors the raw
   float, AUX2 becomes a glitch intensity curve, and AUX3 selects a palette slot
   defined in `mapping.leds.palette`.
3. **LED driver host script** (usually the same Raspberry Pi) opens a second
   serial port pointed at the Teensy. Whenever AUX3 changes, it blasts a `C r g b`
   command using the palette color. AUX2 feeds the `I x` command for intensity.
   The Teensy firmware multiplies the two so operators get palette-stable fades
   instead of thrashing RGB values.
4. **Operator intuition**: high crowd sync = hotter palette slot (think neon
   pink or gold) and higher intensity. Low crowd energy settles back into the
   cool blues. Because the serial protocol is dead-simple text, you can monitor
   it with `screen /dev/ttyACM0 115200` and manually inject overrides during a
   show if the automation gets weird.

## Cable length + mounting notes

* Keep the Teensy within 30 cm of the first LED if you’re running raw data. If
  you need longer, add a logic-level shifter or buffer board.
* Mount the electrolytic capacitor right at the strip input; long leads defeat
  the point.
* Secure the strip tail so that impact loads go into the airframe, not the
  solder tabs. Gaffer tape plus a dab of E6000 glue keeps things punk-proof.

Now go light the sky — responsibly chaotic, zero smoke.
