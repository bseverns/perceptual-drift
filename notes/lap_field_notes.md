# Laptop / studio notes

> For sessions on the dev laptop or studio rig. Keep it scrappy but specific.

## Session template
- Date/time:
- Location / environment:
- Scene name + config:
- Hardware context (camera, MIDI/OSC targets):
- Observations:
- Glitches / failures:
- Hypotheses / next experiments:
- Artifacts (logs, screenshots):

## Sample sessions
### 2024-05-18 — home desk
- Scene: `jetson_hello_camera.py` @ `config/platform_desktop.yaml`
- Observations: drift ~0.005 when still; rises to 0.2 with hand waves.
- Glitches: window tearing at 60 fps on iGPU; capped to 45 fps for stability.
- Next: test OSC routing into Ableton via `config/mappings/osc.yaml`.

### 2024-05-25 — rehearsal space
- Scene: same; OSC pointed at lighting console.
- Observations: entropy correlates with fog machine bursts (unexpected win).
- Glitches: Wi‑Fi jitter causing OSC hiccups; consider wired ethernet.
- Next: try MIDI CC mappings and log latency.
