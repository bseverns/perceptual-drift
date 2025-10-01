# Flight Cage Build Log

Welcome to the feral notes for the flight cage. This document is meant to be a teachable blueprint, not a secret handshake. Read it end to end before cutting anything, then keep it open while you work. Highlight the weird stuff, double-check the lengths, and feel free to scribble on the margins IRL.

## Intent + Performance Targets
- **Volume**: a 2.0 m × 2.0 m footprint with a clear internal height of 2.7 m so the rigs can swoop without scalping themselves.
- **Rigidity**: frame deflection under side-load should stay under 10 mm. If you hear a creak, investigate.
- **Containment**: net gaps < 20 mm everywhere, door overlap ≥ 150 mm.
- **Integration ready**: USB umbilicals and kill switches route cleanly to the control pedestal without snag hazards.

## Bill of Materials (BOM)
| Qty | Item | Notes |
| --- | ---- | ----- |
| 8 | 50 mm × 50 mm aluminum extrusion, 2.7 m length | Upright posts; pre-tap the top M8 × 1.25 |
| 8 | 50 mm × 50 mm aluminum extrusion, 2.0 m length | Top frame horizontals |
| 8 | 50 mm × 50 mm aluminum extrusion, 1.95 m length | Bottom frame horizontals (shorter to float inside floor foam) |
| 8 | Corner gusset plates w/ hardware | 6 mm thick, matches extrusion slot |
| 16 | 3-way inside corner cubes | Tie horizontals to verticals |
| 32 | M8 × 20 mm button-head bolts | For extrusion joints |
| 32 | M8 t-nuts | Slide into extrusion slots |
| 12 | 3 mm steel cable, pre-cut to 2.4 m | Net tension perimeter |
| 12 | M6 turnbuckles | Net tension control |
| 1 roll | Knotless nylon net, 25 mm mesh, 3.0 m height × 10 m length | Wraps around frame |
| 1 | Heavy-duty zipper panel, 0.9 m wide × 2.2 m tall | Entry flap core |
| 2 | Neodymium magnet strips, 1 m each | Entry flap auto-close |
| 1 | EVA foam floor tiles, 20 mm thick | Shock absorption |
| 1 | Cable trough, 2 m | Under-door routing for USB bundle |
| 2 | Emergency stop mushroom switches w/ backplates | Mounts on front uprights |
| 1 | USB-C umbilical bundle (5 m) with strain relief grommets | Route through rear upright |
| 1 | Fastener kit for net lacing (paracord, clips) | Keep extras |
| As needed | Gaffer tape, zip ties, heat-shrink | Cleanup |

### Cut List Snapshot
| Component | Quantity | Cut Length | Notes |
| --------- | -------- | ---------- | ----- |
| Vertical posts | 8 | 2700 mm | Deburr both ends |
| Top horizontals | 4 | 2000 mm | North–South runs |
| Top horizontals | 4 | 2000 mm | East–West runs |
| Bottom horizontals | 4 | 1950 mm | North–South, sit inboard |
| Bottom horizontals | 4 | 1950 mm | East–West, leave 25 mm foam reveal |
| Cable brace | 2 | 1800 mm | Optional diagonal in roof plane |

> **Pro-tip**: Label each extrusion with painter’s tape as soon as you cut it. Sharpie “T-N”, “B-E”, etc. saves you from geometry amnesia at midnight.

## Tool Stack
- Drop saw with non-ferrous blade (bring ear protection).
- Deburring tool + flat file.
- Metric hex keys (2.5 mm–6 mm) and torque wrench.
- Cable cutters + swaging tool.
- Rivet gun or screw gun for attaching net tabs.
- Multimeter + continuity buzzer for safety checks.

## Annotated Sketches (ASCII style)
```
Top View (ceiling off)

+---------------------------+
|           USB EXIT ^      |
|                       \   |
|   [Tension Cable]  ( ) \  |  <-- Optional diagonal brace
|                         \ |
| ENTRY FLAP -->  ====     >|
+---------------------------+
```
- **USB EXIT**: rear-right upright, 200 mm above floor. Drill 25 mm pass-through and install grommet for the USB bundle.
- **ENTRY FLAP**: centered on front face; zipper panel + magnet overlap. Foam tiles notch underneath to keep threshold flat.

```
Front Elevation

     kill switch      kill switch
         [ ]              [ ]
          |                |
   +-----------------------------+
   |                             |
   |        NETTING FIELD        |
   |   (laced to perimeter)      |
   |                             |
   +=============================+
           EVA FOAM FLOOR
```
- Mount kill switches at 1.2 m height on both front uprights. Run wiring internally down the extrusion channels.

```
Side Elevation (cutaway)

   Turnbuckle
      /
     /   Net clip
    v     v
   |O----O----O|  Top extrusion
   |           |
   |           |  Rear upright with USB exit
   |   [USB]   |
   |    ||     |
   |    ||     |
   |____||_____|
    Floor foam
```
- Turnbuckles ride on the outer face, aligned with every other net clip to distribute tension.

## Assembly Steps
1. **Lay out the footprint**
   - Snap chalk lines on the floor for a 2.0 m square. Dry-fit the bottom horizontals inside the lines (1950 mm lengths leave 25 mm clearance per side for foam edge).
   - Position corner cubes, but only snug the bolts so adjustments are painless.

2. **Build the base rectangle**
   - Assemble the bottom frame on the floor. Torque to 12 N·m once squareness is confirmed with diagonal measurements (target ≤3 mm difference).
   - Drop the EVA foam tiles inside the frame to make sure the reveal feels right; pull them back out afterwards to avoid coolant spray.

3. **Stand the uprights**
   - Bolt each 2700 mm post into its corner cube. Work opposite corners to keep the base from twisting. Use a buddy or temporary braces—gravity is not your friend here.
   - Plumb each upright with a level; shim under the base if the floor is wavy.

4. **Cap with the top frame**
   - Install the top horizontals, starting with the front edge so you can still walk inside. Use corner gusset plates on the outer faces for shear rigidity.
   - Once all four sides are connected, add the optional roof diagonals if you expect rowdy airflow.

5. **Route the integration hardware**
   - Drill the rear-right upright for the USB grommet 200 mm above floor; deburr aggressively so you don’t shave the cable jacket.
   - Feed the USB bundle from the outside in, leaving a 300 mm service loop on the interior. Secure with strain relief grommet + two zip ties inside the extrusion slot.
   - Mount kill switches on the front uprights at 1.2 m height. Route wiring down the internal channel and out the bottom toward the control pedestal.

6. **Netting + Tension**
   - Hang the net over the frame like a massive curtain. Start clipping at the top rear corner, working clockwise. Maintain 150 mm overlap at the entry face.
   - Lace the perimeter cable through the hem of the net. Install turnbuckles at each midpoint (12 total). Bring the cable taut by hand, then tension each turnbuckle evenly until the net rebounds when tapped.
   - Install magnet strips along the entry flap edges and align with steel keeper plates on the frame. Verify the flap self-closes without drama.

7. **Floor + Door Finish**
   - Reinstall the EVA foam tiles, trimming around the uprights. Add the cable trough under the door to bury the USB bundle. Tape seams so nothing snags.
   - Test the zipper and magnet closure repeatedly. If the flap doesn’t seal, add an extra magnet puck at knee height.

8. **Safety + Functional Checks**
   - **Squareness**: measure diagonals again. If off by >5 mm, loosen, nudge, retighten.
   - **Hardware torque**: run through every bolt with the torque wrench (12 N·m). Paint-pen witness marks so future you can spot loosening.
   - **Kill switches**: continuity test with multimeter; confirm they open the circuit cleanly.
   - **USB path**: plug in the bundle, wiggle it, make sure nothing pinches. Add gaffer pad where the cable exits to prevent chafing.
   - **Net tension**: give the net a hearty shove at mid-span. If it deflects more than 80 mm, tighten the nearest two turnbuckles.

## Integration Touchpoints
- **Rear-right upright**: USB umbilicals enter through a grommet at 200 mm height; route up the extrusion to overhead rigs or down to floor devices. Label the cable bundle with function and date.
- **Front uprights**: kill switches mount on laser-cut backplates that bolt into the extrusion T-slots. Keep leads long enough to swing the switches outward for maintenance.
- **Top frame**: use spare T-slot nuts to mount sensor pods or lighting. Drop a note in the studio notebook whenever you add hardware so the next crew isn’t surprised.
- **Floor edge**: cable trough doubles as a foot stop. If you reroute cables, leave a tag explaining the change. Future-you will thank punk-rock-you.

## Maintenance Log Prompts
- Date / Inspector initials
- Turnbuckle tension status (red-yellow-green)
- Kill switch continuity verified? (Y/N)
- USB sheath intact? (Y/N, notes)
- Any creaks or rattles? (describe the noise and what you did about it)

Keep this README updated after every major tweak. The cage is a living instrument—document every riff so the next fabricator can play it without missing a beat.
