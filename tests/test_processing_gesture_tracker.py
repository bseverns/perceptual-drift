from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TRACKER = (
    ROOT
    / "software"
    / "gesture-tracking"
    / "processing"
    / "PerceptualDrift_Tracker"
    / "PerceptualDrift_Tracker.pde"
)


def _source() -> str:
    return TRACKER.read_text()


def _draw_body(source: str) -> str:
    return source.split("void draw(){", 1)[1].split(
        "void findPresenceBlobs(){", 1
    )[0]


def test_tracker_only_processes_fresh_camera_frames_and_fails_safe():
    source = _source()
    draw = _draw_body(source)

    assert "boolean freshFrame = cam.available();" in draw
    assert "if (!freshFrame)" in draw
    assert "handleCameraStall();" in draw
    assert draw.index("if (!freshFrame)") < draw.index("cam.read();")
    assert 'sendOSC("/pd/consent", 0);' in source
    assert "cameraTimeoutMs" in source


def test_calibration_sends_neutral_heartbeat_and_consent_is_repeated():
    source = _source()
    draw = _draw_body(source)
    consent_sender = source.split("void sendConsentState(){", 1)[1]

    assert "sendSafeOSCFrame();" in draw
    assert (
        "!calibrationMode" in source.split("void refreshConsentGate(){", 1)[1]
    )
    assert 'sendOSC("/pd/consent", consentInt);' in consent_sender
    assert "if (consentInt !=" not in consent_sender


def test_tracker_uses_blob_filtering_and_roi_relative_axis_mapping():
    source = _source()
    draw = _draw_body(source)
    blobs = source.split("void findPresenceBlobs(){", 1)[1]

    assert "int consentCount = centroids.size();" in draw
    assert "blobSamples >= minBlobSamples" in blobs
    assert "centroids.add(" in blobs
    assert "centroidWeights.add(blobSamples)" in blobs
    assert "(cx - consentX) / max(1.0, consentW)" in draw
    assert "(cy - consentY) / max(1.0, consentH)" in draw


def test_consent_requires_persistent_detection_and_roi_stays_onscreen():
    source = _source()
    consent_update = source.split("void updateConsentState(", 1)[1]
    deadline_update = source.split("boolean refreshConsentDeadline(", 1)[1]
    roi_update = source.split("void updateConsentZoneDimensions(){", 1)[1]

    assert "boolean consentArmed=false;" in source
    assert (
        "consentArmed = false;"
        in source.split("void handleCameraStall(){", 1)[1]
    )
    assert (
        "consentArmed = false;"
        in source.split("void captureBackground(PImage frame", 1)[1]
    )
    assert "consentCandidateFrames" in consent_update
    assert "consentCandidateFrames >= consentEnterFrames" in consent_update
    assert "consentReleaseDeadlineMillis = now +" in consent_update
    assert "consentReleaseDeadlineMillis - now > 0" in deadline_update
    assert "consentHoldFrames" not in source
    assert "consentFramesRemaining" not in source
    assert "1.0-consentWRatio" in roi_update
    assert "1.0-consentHRatio" in roi_update


def test_consent_deadline_expires_without_a_fresh_camera_frame():
    source = _source()
    draw = _draw_body(source)
    no_fresh_frame = draw.split("if (!freshFrame){", 1)[1].split(
        "cam.read();", 1
    )[0]

    assert "refreshConsentDeadline(millis())" in no_fresh_frame
    assert "lastConsentCountSent = 0;" in no_fresh_frame
    assert "sendConsentState();" in no_fresh_frame


def test_consent_signaling_cannot_be_disabled_by_a_legacy_switch():
    source = _source()

    assert "consentEnabled" not in source
    assert 'sendOSC("/pd/consent", consentInt);' in source


def test_overhead_camera_can_be_selected_explicitly():
    source = _source()
    selector = source.split("String pickPreferredCamera", 1)[1]

    assert "preferredCameraIndex" in selector
    assert "preferredCameraContains" in selector
    assert 'entry.indexOf("usb")' in selector
    assert 'entry.indexOf("facetime")' in selector
