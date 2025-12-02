# perceptual-drift architecture

> What this is: a working map of how signals flow through the project. Half studio notebook, half teaching guide.

## Core idea
Perceptual-drift listens to an environment, measures how it changes, and maps that change onto instruments (visual, audio, kinetic). The pipeline is intentionally modular:

- **Sensors**: cameras today; microphones and other sensors later.
- **Drift-core**: computes metrics like motion, entropy, stability, centers of activity.
- **Mappings**: translates metrics into OSC/MIDI/internal buses.
- **Actuators**: synths, visuals, LEDs, robots, anything that can be driven by those mappings.

## Proposed code layout
Keep IO, computation, and scenes cleanly separated so platform quirks stay contained:

```
src/
  io/
    camera.py       # Capture abstractions (OpenCV, GStreamer, etc.)
    audio.py        # Future: microphone capture
    outputs.py      # Windows, OSC, MIDI, other transports
  drift_core/
    base.py         # CPU reference implementations
    gpu.py          # Jetson / CUDA-accelerated paths
  scenes/
    __init__.py
    hello.py        # Minimal demo scene
config/             # Platform + mapping YAMLs
scripts/            # Setup, profiling, sanity checks
examples/           # Runnable samples (quick and dirty)
hardware/           # Hardware bring-up and wiring notes
notes/              # Field logs and observations
```

## Platform abstraction
Scene code should care about the *what* (metrics, mappings), not the *where* (desktop vs Jetson). Configs drive IO choices.

```python
cfg = load_config(path)
video = io.Camera(cfg["video"])
outputs = io.OutputBus(cfg["outputs"])
drift_engine = drift_core.select_engine(cfg["performance"])

for frame in video:
    metrics = drift_engine.process(frame)
    outputs.update(metrics)
```

- On Jetson, `io.Camera` would wrap a GStreamer pipeline and GPU conversions.
- On desktop, `io.Camera` would default to OpenCV capture.
- Drift-core selection could pick CPU or GPU implementations based on config flags.

## Platform-aware configuration
- `config/platform_desktop.yaml` favors OpenCV, local windows, CPU paths.
- `config/platform_jetson_orin_nano.yaml` leans into GStreamer, headless defaults, and GPU usage.
- Mapping files (`config/mappings/osc.yaml`, `config/mappings/midi.yaml`) keep routing flexible without code changes.

## Future hooks
- **Audio-in**: microphone capture feeding spectral/entropy-based drift metrics.
- **Multi-camera**: fused metrics or per-camera mappings for spatial rigs.
- **Embedded outputs**: LEDs/servos over GPIO, SPI, or I2C for fully embedded installs.
- **Logging + playback**: record drift streams and camera feeds for offline analysis.

Stay modular, keep configs expressive, and let scenes remain small, readable entry points.
