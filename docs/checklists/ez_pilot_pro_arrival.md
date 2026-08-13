# EZ Pilot Pro arrival and factory-state checklist

Status at repository creation: **all physical fields UNVERIFIED**.

Do not reflash or substantially change Betaflight before the received state is documented and archived. Date every artifact and keep it outside the aircraft as well as in the session record.

## As received

- [ ] Photograph aircraft, labels, wiring, antennas, prop orientation, guards, ports, and visible damage.
- [ ] Record package/SKU, serial identifiers, manuals, included battery/charger, connector, cell count, capacity, and measured resting voltage.
- [ ] Identify the flight-controller target, board revision, firmware family/version/build, and bootloader if observable.
- [ ] Save a complete Betaflight CLI/config dump before making changes; hash and back it up.
- [ ] Record receiver type and configuration; do not assume onboard SPI or FrSky D8 from the profile.
- [ ] Record channel order and direction.
- [ ] Record ARM mode/switch and every arming-disable flag.
- [ ] Record Angle/Horizon mode ranges and the actual early-test mode.
- [ ] Record receiver and flight-controller failsafe stages/actions/timing.
- [ ] Record RC endpoints, centers, deadbands, rates, expo, and throttle limits.
- [ ] Record VTX band/channel/power/region settings without transmitting illegally.
- [ ] Record motor/prop/guard condition and stock weight.

## TX16S stock-control characterization

- [ ] Identify the installed TX16S RF module and verify that it supports the required protocol.
- [ ] Create/inspect the EdgeTX model and record EdgeTX version and model backup.
- [ ] Bind the TX16S and confirm receiver identity/range behavior.
- [ ] Verify channel order, direction, centers/endpoints, ARM, flight modes, and failsafe with props removed.
- [ ] Confirm manual control is authoritative with no trainer hardware connected.
- [ ] Perform a stock manual contained-flight characterization before introducing Perceptual Drift influence.
- [ ] Log hover/throttle behavior, trim/drift, Angle-mode response, battery duration/sag, link quality indications, failsafe result, and anomalies.

## Archive record

| Item | Value / artifact path | Evidence label | Operator/date |
| --- | --- | --- | --- |
| Factory CLI/config dump + hash |  | OPERATOR VERIFIED |  |
| FC target/firmware |  | OPERATOR VERIFIED |  |
| Receiver/protocol |  | OPERATOR VERIFIED |  |
| Battery |  | OPERATOR VERIFIED |  |
| Channel order/endpoints |  | OPERATOR VERIFIED |  |
| ARM/modes/failsafe |  | OPERATOR VERIFIED |  |
| VTX configuration |  | OPERATOR VERIFIED |  |
| TX16S RF module/EdgeTX model backup |  | OPERATOR VERIFIED |  |
| Manual-flight characterization |  | FLIGHT TESTED |  |

After this checklist, proceed through the staged gates in [`tx16s-trainer-integration.md`](../hardware/tx16s-trainer-integration.md). Completion does not itself authorize trainer connection or flight.
