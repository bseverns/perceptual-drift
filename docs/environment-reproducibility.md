# Environment Reproducibility

This doc defines the M2 baseline for dependency and environment parity.

## Python dependency policy

- Runtime bundle deps are pinned in:
  - `requirements-starter.txt`
- Dev/CI toolchain deps are pinned in:
  - `requirements-dev.txt`
- Linux/Python 3.10 constraints are centralized in:
  - `constraints/py310-linux.txt`

Install pattern:

```bash
python3 -m pip install -r requirements-dev.txt -c constraints/py310-linux.txt
```

## CI parity

CI runs Python `3.10` and installs from `requirements-dev.txt` with
`constraints/py310-linux.txt`. Local Linux environments should use the same
command to minimize drift.

## Starter container (desktop/lab reproducibility)

A reference image is provided:

`Dockerfile.starter`

Build and run:

```bash
docker build -f Dockerfile.starter -t perceptual-drift-starter .
docker run --rm -it perceptual-drift-starter
```

Use this image for dependency-parity checks and quick onboarding loops where
camera/serial hardware passthrough is not required.

## Jetson Orin Nano profile

Profile file:

`config/env/jetson-orin-nano.profile.yaml`

Bootstrap script:

`scripts/setup_jetson.sh`

The script installs apt packages from:

`config/env/jetson-apt-packages.txt`

and Python dependencies from:

- `requirements-starter.txt`
- `constraints/py310-linux.txt`

## Raspberry Pi profile

Bootstrap script:

`scripts/setup_pi.sh`

Apt baseline:

`config/env/pi-apt-packages.txt`

Python install path matches starter pins + constraints.

## Environment manifests

Capture and archive environment snapshots after each fresh setup:

```bash
./scripts/export_env_manifest.sh
```

Output goes to `runtime/env_manifest_<timestamp>.txt` by default.

