import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import scripts.validate_config as validate_config


def test_validate_defaults_pass():
    exit_code = validate_config.main(
        [
            "--mapping",
            str(REPO_ROOT / "config" / "mapping.yaml"),
            "--recipes",
            str(REPO_ROOT / "config" / "recipes"),
            "--quiet",
        ]
    )
    assert exit_code == 0


def test_validate_flags_bad_config(tmp_path: Path):
    bad_map = tmp_path / "bad.yaml"
    bad_map.write_text(
        """
osc:
  port: 1234
  address_space:
    altitude: "/bad/alt"
    lateral: "/bad/lat"
    yaw: "/bad/yaw"
    crowd: "/bad/crowd"
    consent: "/bad/consent"
mapping:
  altitude:
    deadzone: 1.5
    gain: 1.0
    curve: "linear"
  lateral:
    deadzone: 0.0
    gain: 1.0
    curve: "linear"
  yaw_bias:
    bias: 0.0
    jitter: 0.0
  glitch_intensity:
    base: 0.0
    max: 1.0
bridge:
  modes:
    smooth:
      aux_strategy: full
      neutral_rc: false
      jitter_scale: 1.0
"""
    )

    exit_code = validate_config.main(
        ["--mapping", str(bad_map), "--recipes", str(tmp_path), "--quiet"]
    )
    assert exit_code == 1
