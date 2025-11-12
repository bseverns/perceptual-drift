# Perceptual Drift pre-show smoke test

The whole point of this ritual is to catch busted control-stack plumbing *before*
we unleash the drone over humans. Treat it like a soundcheck for the entire
installation: OSC crowd tracker, MSP flight pipe, LED shimmer, and DSP drone.

## Why this exists

* The Processing tracker can fall over when a laptop camera driver silently
  changes formats.  We replace the lens with prerecorded motion so the math
  still gets exercised.
* Serial adapters wiggle loose.  The mocks here verify we can speak the same
  dialect as the Teensy LED and DSP boards without waking the actual hardware.
* Betaflight expects MSP packets with exact framing.  If we violate the XOR
  checksum, we find out now instead of watching props refuse to spin in front
  of an audience.

## Gear + files you need

* Python 3.10+ with `python-osc` and `pyyaml` installed (same stack as the
  control bridge).
* This repo checked out locally so the script can load:
  * `config/mapping.yaml`
  * `config/test-fixtures/gesture_fixture_frames.json`
  * `software/control-bridge/osc_msp_bridge.py`
* Optional but nice: the Processing IDE open to
  `software/gesture-tracking/processing/PerceptualDrift_Tracker/` so you can
  skim the comments while the smoke test runs. It reads like a lab notebook on
  purpose.

## Quickstart: run the automated check

```bash
./scripts/check_stack.py
```

What happens under the hood:

1. A loopback MSP serial link spins up so we can inspect what `osc_msp_bridge`
   would have blasted at the flight controller.
2. The camera fixture stored in `config/test-fixtures` gets replayed frame by
   frame.  It mirrors the Processing thresholding logic, so if we broke the math
   we’ll see bogus OSC values.
3. Fresh OSC packets feed the bridge which, in turn, emits MSP frames.  The
   script cracks them open and asserts the header, checksum, and channel ranges
   match what Betaflight wants.
4. LED and DSP mocks get pinged.  They expect the same newline-terminated
   commands our Teensy sketches do and cough up `*-HEARTBEAT` responses.  No
   heartbeat means you should check your USB cables before soundcheck.

If everything passes you’ll see three ✅ lines followed by “Go run the real rig.”
Anything else? Start reading the tracebacks — they’re intentionally opinionated
about what went sideways.

## Manual fallback (when you want to poke each lane by hand)

1. **Processing tracker sanity:** Launch the sketch, hit play, and point it at a
   looping video from the fixture folder.  Confirm the HUD text is changing and
   the consent bar goes green when you mash the space bar.
2. **OSC inspection:** Run `./scripts/check_stack.py --osc-port 9101` so the
   automated harness leaves port 9000 free.  Fire up `oscdump localhost 9101`
   (from `liblo` or similar) while the script runs to see the normalized floats
   fly by.
3. **Serial sniffing:** Plug the real Teensy LED board in parallel with the
   mock.  Send `echo "PING" > /dev/ttyACM*` and confirm the heartbeat message.
   Repeat for the DSP port.  You’ll hear the Mozzi patch breathe when it’s happy.
4. **Betaflight double-check:** Open Betaflight Configurator, connect to the FC,
   and watch the receiver tab while the script streams.  The channels should
   twitch in sync with the fixture’s pretend crowd wave.

## Resetting after a failure

*Power cycle anything that touched USB* — Teensy boards get grumpy about stale
serial sessions.  Then re-run the script.  If MSP packets still fail the
checksum, yank the repo back to `main` and diff your local changes to the bridge
or mapping YAML.

Stay scrappy, stay safe, and keep the drone pointed away from faces.
