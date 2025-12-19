# OBS telemetry overlay (battery + RSSI)

This is the “don’t lose the link” overlay: keep pilots and safety spotters
watching battery + RSSI without ducking into Betaflight OSDs. Build the scene
once, stash it in your OBS Scene Collection, and reuse it on both the ground
station laptop and Pi/Jetson mirrors.

## Quick start: poll telemetry into OBS-readable files
1. Run the poller beside OBS (same laptop as `software/video-pipeline` is ideal):
   ```bash
   python3 scripts/obs_telemetry_overlay.py \
     --url http://GROUND-STATION:9750/telemetry \
     --out-dir runtime/obs_overlay \
     --hz 2.5
   ```
   * Outputs: `runtime/obs_overlay/telemetry_overlay.txt` (for Text sources) and
     `telemetry_overlay.json` (for Browser/Text+GDI overrides).
   * The script accepts `voltage`/`vbat` and `rssi`/`link_quality` keys so it can
     slurp Betaflight OSD relays, MSP mirrors, or ROS shims without edits.
   * Default refresh is 2 Hz; bump `--hz` if you want snappier updates.

2. Keep the poller running while OBS is open so the file timestamps keep
   moving—operators can see staleness in the JSON.

## Build the OBS scene (capture → queue → color → telemetry)
Create a new scene per preset so latency math mirrors `config/video-presets.json`
plus `software/video-pipeline/gst_launch.sh`.

1. **Video Capture Device** named after your preset (e.g. `capture_delayed_glitch`).
   Match the `device` from the preset and disable buffering in source properties
   for the low-latency chains.
2. **Queue/delay filter**
   * `clean_low_latency`: no delay filter.
   * `delayed_glitch`: `Video Delay (Async)` ≈ 6000 ms (180 frames @ 30 fps).
   * `riot_feedback`: `Video Delay (Async)` ≈ 8000 ms (240 frames @ 30 fps).
   * `ambient_soften`: no extra delay—just mirror the leaky queue by keeping
     buffering off.
3. **Color/contrast filter** (`Color Correction` filter in OBS)
   * Map values directly from the preset’s `videobalance` block. Examples:
     * `delayed_glitch`: Saturation = 1.6, Brightness = -0.05.
     * `ambient_soften`: Saturation = 0.85, Brightness = 0.04, Hue = 0.02.
     * `mirror_twins`: Saturation = 1.2, Contrast = 1.1.
     * `riot_afterburn`: Saturation = 1.9, Contrast = 1.4, Brightness = -0.06.
   * Keep the filter order capture → delay → color so you can count latency like
     the GStreamer chain.
4. **Telemetry Text source** (on top of the stack)
   * Source type: Text (GDI+ on Windows) or FreeType2 on Linux.
   * Enable “Read from file” and point to
     `runtime/obs_overlay/telemetry_overlay.txt`.
   * Styling for 10 ft legibility: heavy sans font at 36–48 px, bold, white
     text, 2–3 px drop shadow, and a subtle dark background. Anchor bottom-right
     if you want pilots to glance near the OSD position.
5. **Optional HTML/Browsers**: If you prefer a Browser source, load
   `runtime/obs_overlay/telemetry_overlay.json` via `file:///…` and render it
   with your own CSS—keep the same white-on-dark contrast.

Once you like it, export the Scene Collection. Name the scene itself after the
chain (examples in the recipes) so crew can pick the right one mid-show.

## Ground station / Pi mirror notes
* Run the same poller on Jetson/Pi if they’re the ones talking to the telemetry
  endpoint; OBS on the laptop can still read the files over a network share.
* `scripts/record_fpv.sh` is handy for proving the overlay matches the captured
  feed—kick off a quick recording while the poller runs and review the MP4.
