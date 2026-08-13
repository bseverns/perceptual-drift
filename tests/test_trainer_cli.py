from software.trainer_control import cli


def test_dry_run_emits_exact_requested_frames(capsys):
    assert cli.main(["--frames", "2"]) == 0
    lines = capsys.readouterr().out.splitlines()
    assert len([line for line in lines if line.startswith("{")]) == 2
    assert lines[-1] == "STATE: SAFE — ZERO ALGORITHMIC AUTHORITY"
