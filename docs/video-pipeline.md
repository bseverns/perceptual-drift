# Video pipeline map (VRX → GStreamer/OBS → projection)

This is the fast orientation for how camera/VRX capture lands inside our projections. Treat it like a field notebook: short map, one command, one preset.

```mermaid
flowchart LR
    subgraph Capture
        A[Camera/VRX]
    end
    subgraph Processing[Video processing]
        B[gst-launch via<br>software/video-pipeline/gst_launch.sh]
        C[OBS Scene Collection
(mirrors same chains)]
    end
    subgraph Output
        D[Projector / display]
    end
    A -->|USB capture / V4L2| B
    B -->|live or delayed feed| C
    C --> D
    subgraph Config[Preset source]
        E[config/video-presets.json]
    end
    E -. selects GST chain .-> B
```

## Worked example: clean, low-latency monitor

1. Launch the preset with your capture device (swap `/dev/video1` for your card):

   ```bash
   ./software/video-pipeline/gst_launch.sh clean_low_latency --device /dev/video1
   ```

2. The shell preset matches the `clean_low_latency` entry inside `config/video-presets.json` (keep both in sync when you tune it):

   ```json
   "clean_low_latency": {
     "description": "Display FPV with minimal processing",
     "gst": "v4l2src device=/dev/video0 ! videoconvert ! autovideosink sync=false"
   }
   ```

   The script’s built-in case statement uses the same chain and swaps the device to your CLI value (or `$VIDEO_DEVICE`) before firing `gst-launch-1.0`. OBS scenes mirror the same intent—add a Video Capture Source that targets the same device and reuse the color/buffer steps if you prefer an OBS-only workflow. This keeps `config/video-presets.json` the teaching doc while `gst_launch.sh` stays the quick-launch hammer.

## Tips for remixing presets
- Copy/paste any `gst` string from `config/video-presets.json` into the launcher, then tweak saturation/queue values live in rehearsal.
- When experimenting in OBS, keep the same ordering: capture → optional queue/buffer → color/balance nodes → sink/projector. Matching structure keeps latency math consistent.
- New moods? Add a preset to `config/video-presets.json`, then run `./software/video-pipeline/gst_launch.sh <new_name>`. Pin that chain in a recipe if you need it bundled with control/LED settings.
