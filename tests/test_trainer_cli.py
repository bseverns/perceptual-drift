from software.trainer_control import cli, preflight


def test_dry_run_emits_exact_requested_frames(capsys):
    assert cli.main(["--frames", "2"]) == 0
    lines = capsys.readouterr().out.splitlines()
    assert len([line for line in lines if line.startswith("{")]) == 2
    assert lines[-1] == "STATE: SAFE — ZERO ALGORITHMIC AUTHORITY"


def test_preflight_read_only_serial_probe_observes_heartbeats(
    monkeypatch, capsys
):
    class FakeSerial:
        def __init__(self):
            self.lines = [b"HB,1\n", b"HB,2\n", b"HB,3\n"]
            self.writes = []
            self.closed = False

        def readline(self):
            return self.lines.pop(0) if self.lines else b""

        def write(self, payload):
            self.writes.append(payload)

        def close(self):
            self.closed = True

    serial_port = FakeSerial()
    monkeypatch.setattr(
        preflight.serial, "Serial", lambda *_args, **_kwargs: serial_port
    )

    assert preflight.main(["--serial", "/dev/fake"]) == 0
    output = capsys.readouterr().out
    assert "SOFTWARE OBSERVED: trainer bridge responding" in output
    assert "HARDWARE SIGNAL: still UNVERIFIED" in output
    assert serial_port.writes == []
    assert serial_port.closed is True
