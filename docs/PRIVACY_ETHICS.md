# PRIVACY & ETHICS

We promised participants that the rig never spies, never leaks, and always
broadcasts its intent. Hold us to it.

## Consent-forward interface contract

Idle vs engaged has to be painfully obvious to anyone glancing at the rig.
Coordinate these signals before powering the drone:

- **Processing tracker HUD** — The consent bar drawn in the Processing sketch
  (`software/gesture-tracking/processing/PerceptualDrift_Tracker/PerceptualDrift_Tracker.pde`)
  stays **amber with "CONSENT: OFF" text** while idle, then flips to **cyan with
  "CONSENT: ON"** when the facilitator taps the space bar (`keyReleased()` block).
- **Projector overlay** — When consent is low the silhouette output fades toward
  grayscale (see `docs/experience/README.md` onboarding table). Once consent is
  granted, motion blooms into full-spectrum color and latency readouts unhide.
- **Bridge console** — The OSC→MSP bridge slams a neutral RC frame (centered
  sticks, low throttle, chill AUX) whenever `/pd/consent` drops (see the neutral
  frame handling in `software/control-bridge/osc_msp_bridge.py`). The terminal prints
  `consent=0` heartbeat lines while idle and `consent=1` when packets are armed.
- **Physical LEDs** — AUX-driven fixtures should mirror the same amber→cyan state
  as the Processing HUD via the consent channel described in
  `config/mapping.yaml`. If your LED controller can’t show both states, it does
  not ship.

### Operator cues

1. **Idle (consent = 0)**
   - Processing status bar: amber, “CONSENT OFF”.
   - OSC bridge shell: shows neutral RC values (`1500` µs) thanks to the chill
     payload fallback.
   - LEDs: amber pulse at 30% brightness.
   - Projection: grayscale silhouettes; “System observing only” footer.
2. **Engaged (consent = 1)**
   - Processing status bar: cyan, “CONSENT ON”.
   - OSC bridge shell: live RC packets stream at ~50 Hz.
   - LEDs: cyan sweep pattern at 60% brightness.
   - Projection: full color, latency ticker visible.

## Buffer purge drill

When anyone asks for deletion, drop everything and purge the volatile caches.

1. **Key combo**: With the Processing window focused, hit `Shift` + `Backspace`
   to clear the current participant text overlays. Immediately switch to the
   operator terminal and run the command below.
2. **Command**: `./scripts/purge_buffers.sh`
   - This script wipes the runtime caches in `$PERCEPTUAL_DRIFT_RUNTIME` (defaults
     to `~/perceptual-drift-runtime`) and `software/video-pipeline/tmp` before
     recreating empty, locked-down directories.
   - It also drops a JSONL event into `logs/ops_events.jsonl` with the exact
     directories it touched plus your operator ID (override with `OPERATOR_ID`).
3. **Verification**: Tail the last three log entries to prove the wipe happened
   (pipe through `jq` if you want pretty output):
   ```bash
   tail -n 3 logs/ops_events.jsonl | jq '.'
   ```
4. **Verbal confirmation**: Tell the participant exactly what you just erased.
   Example script: “Buffers are nuked; nothing left in cache, nothing leaves this
   room.”

## Event logging expectations

- `logs/ops_events.jsonl` is the canonical ledger. It inherits `.gitignore`
  rules so it never syncs upstream.
- Automation now covers the usual suspects: `scripts/purge_buffers.sh`,
  `scripts/record_fpv.sh`, and the OSC→MSP bridge all append structured events
  with timestamps, hostnames, and the current `OPERATOR_ID`.
- Log every consent toggle, buffer purge, manual deletion, and external storage
  connection. When you need a manual entry, echo a JSON blob so the replay tools
  stay happy:
  ```bash
  python3 - <<'PY' >> logs/ops_events.jsonl
  import json, os
  from datetime import datetime, timezone

  event = {
      "timestamp": datetime.now(timezone.utc).isoformat(),
      "operator": os.environ.get("OPERATOR_ID", os.environ.get("USER", "unknown")),
      "host": os.environ.get("HOSTNAME", "unknown_host"),
      "action": "consent_toggle",
      "status": "manual",
      "message": "Stage manager requested hard pause.",
  }
  print(json.dumps(event))
  PY
  ```
- Backup the log to encrypted removable media only after the run sheet is signed
  off. Destroy the media once the show closes.

## Privacy audit replay

When somebody wants receipts, you should be able to play the whole day back
without breaking a sweat.

1. **Collect the ledger** — After each run, package the `logs/` directory with
   `./scripts/seal_logs.sh`. The helper skips empty runs, tars the JSONL ledger,
   and signs the bundle with Minisign so you can prove nobody retroactively
   edited the trail.
2. **Verify the bundle** — On a clean machine, run:
   ```bash
   minisign -V -P <team-public-key> \
     -m logs/bundles/privacy_audit_<timestamp>.tar.gz \
     -x logs/bundles/privacy_audit_<timestamp>.tar.gz.minisig
   ```
   The command should shout “Signature and comment signature verified.” before
   you proceed. See `docs/safety/LOG_SIGNING.md` for the gritty setup details.
3. **Review the timeline** — Extract the tarball into a quarantine folder and
   run your favorite JSONL tool (`jq`, Datasette, or plain old `less`). Verify
   that every purge, consent flip, FPV recording, and shutdown handshake matches
   the show report. If an action is missing, the run is **not** compliant.

Compliance means every visitor-triggered action is accounted for, the Minisign
signature validates, and the ledger shows the buffers dying before any storage
device leaves the room. Anything less? Pause the show and figure out what went
sideways.

## Public signage (print + screen reader copy)

> **Perceptual Drift — What’s Happening Here?**
>
> This drone only reacts while the cyan lights are on. Amber lights mean it’s
> chilling and not saving anything. No photos, names, or audio leave this room.
> Ask any crew member to pause or erase your data on the spot.
>
> Accessibility notes: Text is 18pt minimum, high-contrast (white on matte
> charcoal). Provide braille/raised-letter placards next to the console and offer
> the same copy in large-print handouts. Screen-reader script mirrors this text
> and explicitly calls out the color states for low-vision guests.

## Storage retention policy

- **Live video + motion buffers**: RAM-only, purged via `purge_buffers.sh` on
  every consent drop or shutdown. No files persist on disk.
- **Telemetry**: OSC and MSP streams stay on the LAN, never written to disk.
- **Analytics**: Aggregate counts (e.g., visitor tally) stored in plaintext notes
  with no timestamps. Destroy notes weekly.
- **Backups**: Only configuration files and code live in version control. No
  runtime data touches Git, cloud sync, or shared drives.

## Checklist: keep data on the local net

Before opening doors each day:

1. `nmcli radio wifi off` — Wi-Fi radios stay dark.
2. `ip addr` — Confirm only the wired /24 subnet is active.
3. `sudo tcpdump -ni <iface> 'host not 239.0.0.0/8'` for 60 seconds — Verify no
   outbound traffic beyond multicast discovery.
4. `sudo lsof -i` — Ensure only OSC (`:9000`) and MSP serial endpoints are open.
5. Confirm the router/firewall ACL blocks WAN egress for the rig’s MAC addresses.
6. Simulate a full run while monitoring with `iftop`. No spikes beyond the LAN =
   green light.
7. Document the check in `logs/ops_events.jsonl`.

## Related files to enforce these promises

- **Consent channel source** —
  `software/gesture-tracking/processing/PerceptualDrift_Tracker/PerceptualDrift_Tracker.pde`
  defines the `/pd/consent` toggle and HUD colors.
- **OSC bridge behavior** —
  `software/control-bridge/osc_msp_bridge.py` shows how consent gates RC output
  and where to extend neutral handling.
- **Operator choreography** — `docs/control-stack-playbook.md` and
  `docs/experience/README.md` outline the human-facing flow; keep them in sync
  when you tweak the cues above.

Punk-rock ethos: Transparency or bust. If you can’t explain every byte’s path to
someone’s grandma, the rig doesn’t go live.
