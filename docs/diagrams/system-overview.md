# System Overview (Mermaid)

Welcome to the loud-and-clear map of how Perceptual Drift actually moves, thinks, and glows. This page is meant to be a teaching tool, a living notebook, and a reminder that magic is just a stack of deterministic steps with a bit of synthwave swagger.

## Whole-System Signal Highway

```mermaid
flowchart LR
    subgraph Audience
        AudienceGestures["Gestures & Body Motion"]
    end
    subgraph ProcessingTracker[Processing Vision Tracker]
        Cameras["IR / RGB Cameras"]
        VisionProcessing["Processing Sketch — Vision Processing"]
        AudienceGestures --> Cameras
        Cameras --> VisionProcessing
        VisionProcessing -->|OSC| GestureMapper
    end
    subgraph ControlStack[Drone System]
        GestureMapper -->|MSP via UART| FlightController["Betaflight Flight Controller"]
        FlightController --> Motors["Drone Motors"]
        FlightController --> LedDriver["Addressable LED Driver"]
        LedDriver --> DroneLeds["Onboard LED Array"]
        Motors --> DroneMotion["Drone Motion"]
        DroneMotion -->|FPV Analog| GroundRx["Video RX → USB Capture"]
    end
    subgraph Projection
        GroundRx -->|GStreamer → OBS| VideoMix["Processed Video Mix"]
        VideoMix --> Projectors["Projectors / Displays"]
    end
```

The flow above is the bird’s-eye view: humans throw gestures, the Processing sketch chews them into signals, the mapping layer translates vibe into control packets, and the drone both flies and beams video back for remixing.

## Stepwise Breakdown: Gesture to Flight

```mermaid
sequenceDiagram
    participant Performer
    participant Camera
    participant VisionApp as Vision App (Processing)
    participant Mapper as Gesture→Control Mapper
    participant FC as Flight Controller
    participant Drone as Drone Platform

    Performer->>Camera: Move / Gesture / Position Shift
    Camera->>VisionApp: Sensor Frames (IR + RGB)
    VisionApp->>VisionApp: Feature Extraction + Gesture Classification
    VisionApp-->>Mapper: OSC Packet (gesture event + metadata)
    Mapper->>Mapper: Map gesture → MSP command set
    Mapper-->>FC: UART MSP Messages
    FC->>Drone: Motor PWM + LED Patterns
    Drone-->>Performer: Motion + Light Feedback
```

**Teach-back checklist:**

1. Cameras are the first translators, turning vibes into pixels.
2. The Processing vision sketch chews those pixels into actionable gesture tags. The openFrameworks fork was an experiment now living in off-repo archives, so treat Processing as the reference build unless you revive that history on purpose.
3. Mapping code turns gestures into multi-field MSP packets (throttle, roll, LED mode, etc.).
4. Betaflight consumes MSP, applies PID loops, and drives both motors and LEDs.
5. The performer sees light, hears motors, and keeps the feedback loop alive.

## Stepwise Breakdown: Airborne Feedback to Projection

```mermaid
sequenceDiagram
    participant Drone as Drone FPV Cam
    participant VTx as Analog VTx
    participant VRx as Ground VRX + Capture
    participant GStreamer
    participant OBS
    participant Projector

    Drone-->>VTx: Live analog FPV feed
    VTx-->>VRx: 5.8 GHz analog signal
    VRx-->>GStreamer: Digitised video frames
    GStreamer->>GStreamer: Color correction + latency trim
    GStreamer-->>OBS: Clean feed (NDI/virtual cam)
    OBS->>OBS: Composite overlays + performer cues
    OBS-->>Projector: Final stage visuals
```

**Notes for operators who like to break rules responsibly:**

* GStreamer is where you shave latency and fix nasty RF artifacts.
* OBS is the playground for overlays, safety warnings, and live annotations.
* If latency spikes, re-check USB capture bandwidth before blaming the drone.

## Ops & Safety Event Ladder

```mermaid
flowchart TD
    A[Preflight Checklist Signed] --> B[Hardware Power-On]
    B --> C{Self-Test Pass?}
    C -->|No| C1[Abort → Troubleshoot Sensors]
    C -->|Yes| D[Link Vision App ↔ Mapper ↔ FC]
    D --> E{MSP Heartbeat Stable?}
    E -->|No| E1[Reboot Mapper → Restore UART]
    E -->|Yes| F[Arm Drone in Safety Cage]
    F --> G{Showtime Window Open?}
    G -->|No| G1[Hold Hover Tests]
    G -->|Yes| H[Launch Performance Loop]
    H --> I{Unexpected State?}
    I -->|Yes| I1[Fail-safe: Disarm, Cut LEDs]
    I -->|No| J[Postflight Log Capture → Archive]
```

This ladder keeps the crew honest: no jazzing the drone until checklists sing, and every anomaly has a pre-decided escape hatch.

## Remix Ideas & TODOs

* Layer MIDI input on the mapper for hybrid performances.
* Add telemetry overlay (battery, RSSI) in OBS so operators can catch drift early.
* Prototype a “ghost mode” where gestures record to buffer before taking control.

Own the stack, document the weirdness, and keep iterating.
