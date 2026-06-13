from pathlib import Path

import pytest

import scripts.validate_config as validate_config

REPO_ROOT = Path(__file__).resolve().parents[1]


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


def test_validate_flags_bad_consent_default_state(tmp_path: Path):
    bad_map = tmp_path / "bad_consent.yaml"
    bad_map.write_text(
        """
osc:
  port: 1234
  address_space:
    altitude: "/pd/alt"
    lateral: "/pd/lat"
    yaw: "/pd/yaw"
    crowd: "/pd/crowd"
    consent: "/pd/consent"
consent:
  default_state: 2
mapping:
  altitude:
    deadzone: 0.1
    gain: 1.0
    curve: "linear"
  lateral:
    deadzone: 0.1
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


def _run_validate_and_capture(args, capsys):
    exit_code = validate_config.main(args)
    captured = capsys.readouterr()
    return exit_code, captured.out + captured.err


@pytest.mark.parametrize(
    ("filename", "body", "expected_bits"),
    [
        (
            "bad_deadzone.yaml",
            """
osc:
  port: 1234
  address_space:
    altitude: "/pd/alt"
    lateral: "/pd/lat"
    yaw: "/pd/yaw"
    crowd: "/pd/crowd"
    consent: "/pd/consent"
mapping:
  altitude:
    deadzone: 1.5
    gain: 1.0
    curve: "linear"
  lateral:
    deadzone: 0.1
    gain: 1.0
    curve: "linear"
bridge:
  modes:
    smooth:
      aux_strategy: full
      neutral_rc: false
      jitter_scale: 1.0
""",
            [
                "mapping.altitude.deadzone",
                "Received: 1.5",
                "Allowed:",
                "Example fix:",
            ],
        ),
        (
            "bad_curve.yaml",
            """
osc:
  port: 1234
  address_space:
    altitude: "/pd/alt"
    lateral: "/pd/lat"
    yaw: "/pd/yaw"
    crowd: "/pd/crowd"
    consent: "/pd/consent"
mapping:
  altitude:
    deadzone: 0.1
    gain: 1.0
    curve: "banana"
  lateral:
    deadzone: 0.1
    gain: 1.0
    curve: "linear"
bridge:
  modes:
    smooth:
      aux_strategy: full
      neutral_rc: false
      jitter_scale: 1.0
""",
            [
                "mapping.altitude.curve",
                "Received: 'banana'",
                "Allowed:",
                "'linear'",
                "'expo'",
                "Example fix:",
            ],
        ),
        (
            "missing_consent_route.yaml",
            """
osc:
  port: 1234
  address_space:
    altitude: "/pd/alt"
    lateral: "/pd/lat"
    yaw: "/pd/yaw"
    crowd: "/pd/crowd"
mapping:
  altitude:
    deadzone: 0.1
    gain: 1.0
    curve: "linear"
  lateral:
    deadzone: 0.1
    gain: 1.0
    curve: "linear"
bridge:
  modes:
    smooth:
      aux_strategy: full
      neutral_rc: false
      jitter_scale: 1.0
""",
            [
                "osc.address_space.consent",
                "Received: missing",
                "Allowed:",
                "Example fix:",
            ],
        ),
        (
            "bad_default_state.yaml",
            """
osc:
  port: 1234
  address_space:
    altitude: "/pd/alt"
    lateral: "/pd/lat"
    yaw: "/pd/yaw"
    crowd: "/pd/crowd"
    consent: "/pd/consent"
consent:
  default_state: 2
mapping:
  altitude:
    deadzone: 0.1
    gain: 1.0
    curve: "linear"
  lateral:
    deadzone: 0.1
    gain: 1.0
    curve: "linear"
bridge:
  modes:
    smooth:
      aux_strategy: full
      neutral_rc: false
      jitter_scale: 1.0
""",
            [
                "consent.default_state",
                "Received: 2",
                "Allowed:",
                "Example fix:",
            ],
        ),
        (
            "bad_aux_strategy.yaml",
            """
osc:
  port: 1234
  address_space:
    altitude: "/pd/alt"
    lateral: "/pd/lat"
    yaw: "/pd/yaw"
    crowd: "/pd/crowd"
    consent: "/pd/consent"
mapping:
  altitude:
    deadzone: 0.1
    gain: 1.0
    curve: "linear"
  lateral:
    deadzone: 0.1
    gain: 1.0
    curve: "linear"
bridge:
  modes:
    smooth:
      aux_strategy: laser_party
      neutral_rc: false
      jitter_scale: 1.0
""",
            [
                "bridge.modes.smooth.aux_strategy",
                "Received: 'laser_party'",
                "Allowed:",
                "'full'",
                "'crowd_only'",
                "Example fix:",
            ],
        ),
    ],
)
def test_validate_operator_facing_errors_for_common_mapping_failures(
    tmp_path: Path, capsys, filename: str, body: str, expected_bits: list[str]
):
    bad_map = tmp_path / filename
    bad_map.write_text(body)

    exit_code, output = _run_validate_and_capture(
        ["--mapping", str(bad_map), "--recipes", str(tmp_path), "--quiet"],
        capsys,
    )

    assert exit_code == 1
    assert str(bad_map.resolve()) in output
    for bit in expected_bits:
        assert bit in output


def test_validate_recipe_load_failure_is_operator_friendly(
    tmp_path: Path, capsys
):
    bad_recipe = tmp_path / "broken_recipe.yaml"
    bad_recipe.write_text(
        """
name: "Broken Recipe"
control_bridge:
  extends: ../missing_base.yaml
  mapping:
    altitude:
      deadzone: 0.1
      gain: 1.0
      curve: "linear"
    lateral:
      deadzone: 0.1
      gain: 1.0
      curve: "linear"
"""
    )

    exit_code, output = _run_validate_and_capture(
        [
            "--mapping",
            str(REPO_ROOT / "config" / "mapping.yaml"),
            "--recipes",
            str(tmp_path),
            "--quiet",
        ],
        capsys,
    )

    assert exit_code == 1
    assert str(bad_recipe.resolve()) in output
    assert "Key control_bridge" in output
    assert "Problem: recipe could not be loaded" in output
    assert "Received:" in output
    assert "Allowed:" in output
    assert "Example fix:" in output


def test_validate_yaml_parse_failure_is_operator_friendly(
    tmp_path: Path, capsys
):
    bad_map = tmp_path / "parse_error.yaml"
    bad_map.write_text(
        """
osc:
  port: 1234
  address_space:
    altitude: "/pd/alt"
mapping:
  altitude
    deadzone: 0.1
"""
    )

    exit_code, output = _run_validate_and_capture(
        ["--mapping", str(bad_map), "--recipes", str(tmp_path), "--quiet"],
        capsys,
    )

    assert exit_code == 1
    assert str(bad_map.resolve()) in output
    assert "Key <file>" in output
    assert "Problem: file could not be parsed as YAML/JSON" in output
    assert "Received:" in output
    assert "Allowed:" in output
    assert "Example fix:" in output
