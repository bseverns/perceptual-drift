# Minisign log sealing cheat sheet

We use [Minisign](https://jedisct1.github.io/minisign/) because it’s tiny,
auditable, and doesn’t drag in a PKI circus. The goal: every bundle in `logs/`
should be tamper-evident before it hops off the rig.

## 1. Prep the keys

1. Install Minisign on your ops machine (`brew install minisign`, `sudo apt
   install minisign`, or grab the static binary from upstream).
2. Generate a signing key on an offline laptop:
   ```bash
   minisign -G -p team-logs.pub -s team-logs.key
   ```
3. Copy the **public** key (`team-logs.pub`) into the repo’s private ops vault
   and onto any verification machines. Keep the secret key on removable media
   that only travels with the lead operator.
4. Export the secret key path to the runtime so the sealing script can find it:
   ```bash
   export MINISIGN_SECRET_KEY=/secure/usb/team-logs.key
   ```

## 2. Seal a bundle (on the rig)

Run this after you finish purging caches and before anyone walks out with a
thumb drive:
```bash
./scripts/seal_logs.sh
```
The script tars `logs/` (skipping previous bundles), signs it, and logs the event
into `logs/ops_events.jsonl`. If it complains about missing keys, fix that before
proceeding.

## 3. Verify a bundle (off the rig)

1. Copy `logs/bundles/privacy_audit_<timestamp>.tar.gz` *and* the matching
   `.minisig` to a clean review machine.
2. Verify the signature:
   ```bash
   minisign -V -P "$(cat team-logs.pub)" \
     -m privacy_audit_<timestamp>.tar.gz \
     -x privacy_audit_<timestamp>.tar.gz.minisig
   ```
   - Expect “Signature and comment signature verified.”
   - If you see “Signature verification failed,” quarantine the bundle and dig
     into the discrepancy.
3. Untar the archive somewhere quiet and inspect `ops_events.jsonl`. Every run
   should show the purge, consent flips, FPV capture, and shutdown steps in
   order.

## 4. Compliance gut-check

- **Signature valid?** If not, the bundle is toast.
- **Timeline complete?** Missing actions or gaps longer than five minutes need a
  written incident note.
- **Storage hygiene?** Delete the bundle once the audit closes; we only keep the
  signed ledger long enough to satisfy the request.

Stay noisy, stay accountable, and don’t let boring paperwork kill the art.
