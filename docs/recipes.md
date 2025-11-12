# Recipe Playbook — Curating the Drift

Recipes are our shorthand for vibe + tech alignment. Each YAML/JSON file inside
[`config/recipes/`](../config/recipes) binds three systems together:

1. **Gesture → MSP curves** for the OSC→MSP bridge (`software/control-bridge/osc_msp_bridge.py`).
2. **Video feedback chains** for the GStreamer launcher (`software/video-pipeline/gst_launch.sh`).
3. **LED pattern notes** for whoever is driving the Teensy/lighting rig.

Think of a recipe as a field briefing: *what mood are we summoning, what knobs
move, and how should the visuals behave?* This document shows how to author new
recipes without spelunking through code.

---

## Anatomy of a recipe file

```yaml
name: "Soft Consent Lounge"          # Required: human-readable label.
slug: soft-consent-lounge             # Optional: kebab-case ID for docs/URLs.
description: |                        # Required: narrative so operators grok the intent.
  Two-line story for the operator.
intent: "onboarding, whisper mode"   # Optional but encouraged; prints when you load it.

control_bridge:                       # Required: configuration for osc_msp_bridge.py
  extends: ../mapping.yaml            # Optional: merge this recipe with a base map.
  osc:                                # Same schema as config/mapping.yaml
    address_space:
      altitude: "/pd/alt"
      lateral: "/pd/lat"
      yaw: "/pd/yaw"
      crowd: "/pd/crowd"
      consent: "/pd/consent"
  mapping:
    altitude:
      deadzone: 0.08
      curve: "expo"
      gain: 0.55
    lateral:
      deadzone: 0.05
      curve: "linear"
      gain: 0.6
    yaw_bias:
      bias: 0.1
      jitter: 0.05
    leds:
      palette: ["#0d3b66", "#f95738"]
    glitch_intensity:
      base: 0.2
      max: 0.8

leds:                                 # Optional: free-form notes for lighting techs.
  pattern: "slow_breathe"
  notes: |
    Explain how AUX channels map to your firmware. Keep it actionable.

video_pipeline:                       # Optional but recommended: GStreamer cues.
  story: |
    What the projection should feel like.
  default_chain: ambient_soften       # Optional: default chain name for gst_launch.sh.
  gst_chains:
    ambient_soften: |
      v4l2src device="{device}" !
      queue leaky=downstream max-size-buffers=5 !
      videoconvert !
      autovideosink sync=false
    feedback_melt: |
      v4l2src device="{device}" !
      queue max-size-buffers=240 max-size-bytes=0 max-size-time=0 !
      videobalance saturation=1.4 brightness=-0.1 !
      videoconvert !
      autovideosink sync=false
```

### Field notes
- `{device}` inside a chain becomes the capture device path selected at launch.
- `extends` lets you inherit the default curves from [`config/mapping.yaml`](../config/mapping.yaml)
  and only override the spicy bits.
- Every chain string is flattened into a single `gst-launch-1.0` command. Keep
  property spacing tidy; comments are fine, but blank lines get stripped.
- JSON works too if your tooling prefers it, just keep the same keys.

---

## Loading a recipe at runtime

### OSC → MSP bridge
```
python3 software/control-bridge/osc_msp_bridge.py \
  --serial /dev/ttyUSB0 --recipe config/recipes/riot_mode.yaml
```
When a recipe is loaded the script prints the intent line so facilitators know
which mood is live. If the recipe defines `osc.port`, that port automatically
replaces the default unless you supply `--osc_port` yourself.

### Video pipeline launcher
```
./software/video-pipeline/gst_launch.sh --recipe config/recipes/swarm_teaser.yaml
```
You can target a specific chain inside the recipe with a positional argument:
```
./software/video-pipeline/gst_launch.sh --recipe config/recipes/riot_mode.yaml riot_feedback
```
If you omit the chain name the script uses `default_chain`.

---

## Tips for writing your own
- **Lead with intent.** Every recipe starts as a story: who’s in the room,
  what emotional arc are we sculpting, and how risky can the flight feel?
- **Map AUX meaningfully.** AUX1–AUX3 currently carry crowd energy, glitch
  intensity, and LED palette slots. Keep that mapping consistent so lighting
  firmware can swap recipes without reflashing.
- **Version in git.** Recipes are part of the creative score. Commit them so the
  team can diff moods after rehearsals.
- **Test in pairs.** Run the bridge with `--recipe` while a second laptop fires
  the matching `gst_launch.sh --recipe` so you can feel how the vibe locks.

Go wild, but keep the narrative grounded. Novelty is fun; utility keeps the
crowd safe and the drone out of the fog machine.
