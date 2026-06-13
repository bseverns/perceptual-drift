import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from software.operator_ui.state import OperatorState
from software.swarm import swarm_demo

import scripts.check_stack as check_stack

ROOT = Path(__file__).resolve().parents[1]
TRACKER_PATH = ROOT / "software" / "starter-bundle" / "minimal_tracker.py"


def _load_minimal_tracker():
    spec = importlib.util.spec_from_file_location(
        "starter_bundle_minimal_tracker", TRACKER_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(
            f"Unable to load tracker module from {TRACKER_PATH}"
        )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_consent_default_startup_state_is_off_across_subsystems():
    cfg = check_stack.yaml.safe_load(
        (ROOT / "config" / "mapping.yaml").read_text()
    )
    assert cfg["consent"]["default_state"] == 0

    bridge_mapper = check_stack.bridge.Mapper(cfg)
    assert bridge_mapper.state["consent"] == 0

    operator_state = OperatorState(
        base_mapping_path=ROOT / "config" / "mapping.yaml",
        recipes_dir=ROOT / "config" / "recipes",
    )
    assert operator_state.snapshot()["consent_state"] == 0

    swarm_default = swarm_demo.SwarmNode._get_default_consent_state(
        SimpleNamespace(), cfg
    )
    assert swarm_default == 0

    tracker = _load_minimal_tracker()
    with patch.object(sys, "argv", ["minimal_tracker.py"]):
        args = tracker.parse_args()
    assert args.consent_mode == "always_off"
    assert tracker.synthetic_state(0.0, args.consent_mode, 12.0).consent == 0


def test_tracker_camera_bootstrap_sample_starts_with_consent_off():
    tracker = _load_minimal_tracker()

    class FakeCapture:
        def isOpened(self):
            return True

        def read(self):
            return True, object()

        def release(self):
            return None

    class FakeCV2:
        COLOR_BGR2GRAY = object()
        THRESH_BINARY = object()

        def VideoCapture(self, _camera_index):
            return FakeCapture()

        def cvtColor(self, frame, _code):
            return frame

        def GaussianBlur(self, frame, _kernel, _sigma):
            return frame

    with patch.dict(sys.modules, {"cv2": FakeCV2()}):
        camera = tracker.CameraTracker(
            camera_index=0,
            threshold=24,
            consent_threshold=0.1,
            blur_kernel=9,
        )
        first = camera.sample()

    assert first.consent == 0


def test_swarm_consent_normalization_matches_mapping_contract_default():
    cfg = check_stack.yaml.safe_load(
        (ROOT / "config" / "mapping.yaml").read_text()
    )
    behavior_off = swarm_demo._map_gestures_to_behavior(
        {"alt": 1.0, "lat": 1.0, "yaw": 1.0, "crowd": 1.0, "consent": 0.49},
        cfg,
    )
    behavior_on = swarm_demo._map_gestures_to_behavior(
        {"alt": 1.0, "lat": 1.0, "yaw": 1.0, "crowd": 1.0, "consent": 0.5},
        cfg,
    )

    assert behavior_off["consent"] == 0.49
    assert behavior_off["lateral_bias"] == swarm_demo.LATERAL_HOME_M
    assert behavior_off["yaw_bias"] == 0.0
    assert behavior_on["consent"] == 0.5
    assert behavior_on["lateral_bias"] != swarm_demo.LATERAL_HOME_M
