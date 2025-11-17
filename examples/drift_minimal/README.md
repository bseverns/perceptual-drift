# Drift Minimal — your "hello world" drift loop

> Punk-rock version of the install: one image in, three layers out, no drones required.

This folder exists for students who just want to *see* perceptual drift before wrestling with Betaflight, OSC, or GStreamer. `drift_minimal.py` is a Tkinter loop that keeps offsetting the same image with different color washes so you can literally watch a layered feedback vibe in under a minute.

## Run it

```bash
python3 examples/drift_minimal/drift_minimal.py
```

Bring your own picture with `--input path/to/photo.png` if you want to remix rehearsal stills or test pattern cards. Otherwise the script synthesizes a sin/cos gradient at runtime so we don’t rely on binary assets.

## What you get

- **Input**: a single still image (PNG/JPG). Use a webcam grab if you want human silhouettes, or a synth texture if you’re just poking at color.
- **Transform**: three sin/cos modulated offsets that drift at slightly different rates. Each layer gets a tiny color blend so you can feel parallax even with a static source.
- **Output**: a Tkinter window with HUD text (`space` to pause, `q` to quit). Think of it as the smallest reproducible “stack” that still shows layered motion.

## Why it matters

- This is the first stop in the repo syllabus now. Before you flash firmware or rehearse consent drills, run this loop to feel what “drift” even means.
- The code is aggressively commented so you can fork it into Processing, p5.js, TouchDesigner, whatever your class is using.
- No hidden dependencies: Python 3.9+ and Pillow (`pip install pillow`). That’s it.

When you’re ready for the grown-up stack, bounce back to the main README and follow the Control Stack Playbook → Safety Checklist → Experience Playbook sequence.
