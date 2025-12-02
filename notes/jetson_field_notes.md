# Jetson field notes

> Drop observations here when working on/with the Jetson node. Aim for brutal honesty and timestamps.

## Session template
- Date/time:
- Location / environment:
- Scene name + config:
- Hardware context (camera, lens, power):
- Observations:
- Glitches / failures:
- Hypotheses / next experiments:
- Artifacts (logs, photos, captures):

## Sample sessions
### 2024-05-20 — lab dry run
- Scene: `jetson_hello_camera.py` @ `config/platform_jetson_orin_nano.yaml`
- Observations: drift spikes when fluorescent lights flicker; stable around 0.01 when still.
- Glitches: occasional frame drop when HDMI monitor hotplugs.
- Next: pin HDMI, test headless over SSH.

### 2024-05-27 — field install mock
- Scene: same as above; camera pointed at kinetic sculpture.
- Observations: motion center tracks pendulum smoothly; entropy rises with crowd around rig.
- Glitches: thermals creep after 30 minutes; fan curve needs tuning.
- Next: add profiling via `scripts/profile_pipeline.py` and log temps.
