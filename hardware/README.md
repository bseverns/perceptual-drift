# Hardware & Install BOM Cheat Sheet

Welcome to the parts buffet that keeps Perceptual Drift airborne without sacrificing safety cred. This BOM leans on components you can actually source, with specs noted so you can swap equivalents without breaking the vibe. Prices swing, so think in ranges and order spares.

> **Reading tip**: The tables are organized by subsystem. `Qty` means how many you need for a single playable bay (1 drone, 1 projection). Scale linearly for multi-bay builds, or bring a spreadsheet and a stiff coffee.
> **Jetson node callout**: compute is now documented as a first-class citizen. If you’re turning a Jetson Orin Nano into a capture/drift node, pair this BOM with [`hardware/jetson_orin_nano.md`](jetson_orin_nano.md) for flashing, deps, and first-boot rituals.

## Structural & Safety Envelope

| Item | Example part (link) | Key specs | Qty | Notes |
| --- | --- | --- | --- | --- |
| Freestanding net cage | [West Coast Netting custom knotless nylon](https://westcoastnetting.com/knotless-nylon-square-mesh-netting) | 2×2 m footprint, 2.7 m height, 15–20 mm mesh, flame retardant | 1 | Order with stitched perimeter + bungee loops for tension.|
| Cage frame | [Global Truss F24 square truss](https://www.globaltruss.com/products/f24-series/) | 2 m straight sections, 50 mm chords, TUV certified | 6 | Two stacked squares + uprights. Swap for SpeedRail if you already own pipe grid. |
| Floor padding | [ProsourceFit EVA foam tiles](https://www.prosourcefit.com/products/puzzle-exercise-mat) | 24" tiles, 1" thick, ASTM F963 impact rated | 8 | Enough to cover center landing zone + operator path. |
| Entry flap | [MagnaLatch privacy screen kit](https://www.dazoriginal.com/products/heavy-duty-magnetic-screen-door) | 90×210 cm, stitched magnets | 1 | Provides soft entry + automatic closure. Velcro strips tie into net. |
| Safety signage | [Seton ADA-compliant caution sign blanks](https://www.seton.com/caution-watch-for-drones-signs.html) | 300×450 mm, reflective | 2 | Mount at eye level outside each entry, add local language overlay. |
| Operator kill switch TX | [Jumper T-Lite V2](https://www.racedayquads.com/products/jumper-t-lite-v2-controller) | OpenTX/EdgeTX, multiprotocol, momentary switch accessible | 1 | Bind to the drone as an override radio; wrist strap it. |

## Drone Stack

| Item | Example part (link) | Key specs | Qty | Notes |
| --- | --- | --- | --- | --- |
| Micro whoop | [BetaFPV Cetus Pro Kit](https://betafpv.com/products/cetus-pro-fpv-kit) | 75 mm, brushless 1102 18,000 KV, Betaflight 4.x | 1 (+1 spare) | Comes with goggles you can repurpose as backup RX. Swap props for Gemfan 40 mm tri-blade. |
| LiPo packs | [Gaoneng (GNB) 2S 450 mAh HV](https://gnbdpower.com/products/2s-450mah-7-6v-80c-xt30) | XT30, 80C, HV chemistry | 6 | Minimum rotation for 15 min blocks. Label and retire after 80 cycles. |
| Smart charger | [ISDT 608AC](https://www.isdt.co/product/608AC) | 50 W AC/DC, balance charger | 1 | Bring a parallel board (XT30) for field charging. |
| Prop guards | [Gemfan Bumper75 ducts](https://www.gemfanhobby.com/products/bumper75-frame-kit) | 1.5 mm polycarbonate, 40 mm prop clearance | 1 | Swap into Cetus for harder impacts; drill tiny relief for LED cable. |
| Spare props | [Gemfan 1636 tri-blade](https://www.gemfanhobby.com/products/1636-propellers) | 40 mm, 1.5 mm shaft | 10 sets | Assume 2 swaps per show day. |
| Flight controller USB tether | [BetaFPV USB-C extension board](https://betafpv.com/products/usb-c-extension-board) | JST-SH 6-pin to USB-C | 1 | Mount at cage edge so you never dig inside the quad post-crash. |

## Control & Compute

| Item | Example part (link) | Key specs | Qty | Notes |
| --- | --- | --- | --- | --- |
| Control bridge host | [Raspberry Pi 4 Model B 4 GB](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) | Quad-core 1.5 GHz, USB 3.0, 5 V 3 A | 1 | Runs OSC→MSP bridge + logging. Fanless case recommended. |
| Serial adapter | [FTDI USB-UART cable 5V/TTL](https://www.sparkfun.com/products/9717) | 5 V tolerant, 6-pin JST-SH breakout | 1 | Feeds MSP packets direct into FC UART if USB is busy with Betaflight Configurator. |
| Gesture tracking rig | [Intel NUC 11 i5](https://www.intel.com/content/www/us/en/products/details/nuc/kits/nuc11pahi50001.html) | Iris Xe GPU, 16 GB RAM recommended | 1 | Plenty of headroom for OpenCV + Processing. Laptop works if you’re scrappy. |
| Audience camera | [Intel RealSense D455](https://www.intelrealsense.com/depth-camera-d455/) | 90° FOV, depth + RGB, USB 3.1 | 1 | Depth helps isolate crowd motion; fallback to Logitech C920 if budget tight. |
| Network switch | [Netgear GS305 unmanaged](https://www.netgear.com/business/wired/switches/unmanaged/gs305/) | 5× gigabit, fanless | 1 | Tie Pi, NUC, and media server together. |
| UPS | [CyberPower CP600LCD](https://www.cyberpowersystems.com/product/ups/intelligent-lcd/cp600lcd/) | 600 VA, AVR | 1 | Buy yourself 10 minutes to land during a brownout. |

## Video & Projection

| Item | Example part (link) | Key specs | Qty | Notes |
| --- | --- | --- | --- | --- |
| FPV receiver | [ImmersionRC RapidFire module + FatShark Scout HD shell](https://www.immersionrc.com/fpv-products/rapidfire-goggle-module/) | Diversity, 5.8 GHz analog | 1 | Pipe HDMI out to capture. Bring patch + omni antennas. |
| Capture interface | [Magewell USB Capture HDMI Gen 2](https://www.magewell.com/products/usb-capture-hdmi-gen-2) | UVC class, 1080p60, <2 frame latency | 1 | Works cross-platform, rock-solid drivers. |
| Projection surface | [Da-Lite Fast-Fold Deluxe 7.5×10'](https://www.da-lite.com/products/projection-screens/folding-screens/fast-fold-deluxe-screen-system) | 16:9 screen, front or rear surface | 1 | Hang opposite audience for mirrored view. |
| Projector | [Epson Pro EX10000](https://epson.com/For-Work/Projectors/Large-Venue/Pro-EX10000-Wireless-3LCD-Full-HD-1080p-Laser-Projector/p/V11HA35020) | 4500 lumen laser, 16 ms latency | 1 | Laser light engine = minimal maintenance. |
| Signal distribution | [Blackmagic HDMI Micro Converter](https://www.blackmagicdesign.com/products/miniconverters) | Loop out, SDI option | 1 | Splits feed to operator monitor + projector. |
| Operator monitor | [Feelworld FW279](https://feelworld.ltd/portfolio-item/fw279/) | 7", 2200 nit daylight, HDMI | 1 | Mount at console for real-time piloting view. |

## Lights, Audio & Feedback (Optional but Spicy)

| Item | Example part (link) | Key specs | Qty | Notes |
| --- | --- | --- | --- | --- |
| Microcontroller | [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) | 600 MHz Cortex-M7, native USB | 1 | Handles LED animations + Mozzi audio. |
| LED strip | [Adafruit NeoPixel Mini 3535 (60 LED/m)](https://www.adafruit.com/product/2969) | 5 V WS2812B, 60 px/m | 2 m | Wrap drone arms or line cage frame edges. |
| Power regulator | [Pololu 5V 5A step-down U3V70F5](https://www.pololu.com/product/2898) | 6–36 V in, 5 V out 5 A | 1 | Clean power for LEDs from 3S/4S supply. |
| Audio interface | [Focusrite Scarlett 2i2 3rd Gen](https://focusrite.com/en/audio-interface/scarlett/scarlett-2i2) | USB-C, balanced outs | 1 | Routes Mozzi or recorded drone audio into FOH mixer. |
| Powered speakers | [QSC CP8](https://www.qsc.com/live-sound/products/powered-loudspeakers/cp-series/cp8/) | 1000 W Class D, 90° conical | 2 | Mount outside cage pointing inward for spatial coherence. |

## Consumables & Tools

| Item | Example part (link) | Key specs | Qty | Notes |
| --- | --- | --- | --- | --- |
| Hook-and-loop straps | [UMI 12" reusable ties](https://www.amazon.com/dp/B01FZI0C4S) | 12" length, 1" width | 1 pack | Cable management + battery retention. |
| Heat shrink assortment | [3:1 polyolefin kit](https://www.adafruit.com/product/3449) | 1/8"–3/4" | 1 | Use on LED solder joints and USB extensions. |
| Gaffer tape | [Pro Gaff 2" black](https://www.protapes.com/product/pro-gaff/) | Matte cloth, low residue | 3 rolls | Everything is held together with gaff. |
| Fire extinguisher | [Kidde Pro 210](https://www.kidde.com/fire-safety/en/us/products/fire-extinguishers/pro-series/pro-210/) | ABC rated, 2.5 lb | 1 | Park near operator console. |
| First aid kit | [MyMedic The Medic](https://mymedic.com/products/the-medic) | ANSI A compliant | 1 | Refill after every tour leg. |

## Sourcing & Logistics Notes

- **Batch ordering**: For multi-drone fleets, buy duct sets, props, and LiPos in bulk directly from Gemfan and GNB. Customs lead time can hit 4–6 weeks.
- **Documentation binder**: Print the safety checklist, assumption ledger, and this BOM; stash in a waterproof pouch at the operator console.
- **Flight batteries**: Store at 3.8 V per cell; airlines will ask for fireproof LiPo bags ([Bat-Safe Mini](https://batsafe.com/bat-safe-mini/)).
- **Spare drone**: Budget a full ready-to-fly spare per bay. Nothing kills the vibe faster than cannibalizing frames mid-show.

For wiring and cage build specifics, hop into [`flight_cage/`](flight_cage) and [`leds/`](leds). Add your own war stories via pull request — the more field notes, the better.
