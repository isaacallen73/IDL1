# KiCad PCB Design Workflow - Mountain Bike Datalogger

## Overview

This guide walks you through creating your datalogger PCB in KiCad. Estimated time: 4-8 hours for first-time PCB design.

**Difficulty**: Beginner-friendly
**Board complexity**: Medium (multi-device SPI bus, external connectors)
**Layers**: 2-layer (Top + Bottom)

---

## Prerequisites

### Software Installation
1. **KiCad 7.0+** (latest stable): https://www.kicad.org/download/
2. Install with default libraries

### Before You Start
- Read through this entire guide once
- Have your component datasheets handy (for footprint verification)
- Set aside 2-3 hours for your first session

---

## Phase 1: Project Setup (15 minutes)

### Step 1.1: Create New Project
1. Open KiCad
2. `File` → `New Project`
3. Name: `bike_datalogger`
4. Location: `c:\Users\isaac\Documents\Saucy\ESP\Datalogger v2\PCB Design\`

### Step 1.2: Configure Project Settings
1. Open Schematic Editor (click `bike_datalogger.kicad_sch`)
2. `File` → `Schematic Setup`
3. Under "General":
   - Title: "Mountain Bike Datalogger"
   - Revision: "v1.0"
   - Date: (auto)
4. Under "Text Variables":
   - Add `BOARD_SIZE` = "TBD" (will update later)

---

## Phase 2: Schematic Entry (2-3 hours)

### Step 2.1: Add Power Symbols

**Add 3.3V Power Symbol**:
1. Press `A` (Add Symbol)
2. Search: `power:+3V3`
3. Place several around the canvas (you'll need ~8 copies)

**Add Ground Symbols**:
1. Press `A`
2. Search: `power:GND`
3. Place several around canvas

**Add Battery Symbol**:
1. Press `A`
2. Search: `power:VBAT` or `power:Battery_Cell`
3. Place once

### Step 2.2: Add Main Components

#### XIAO ESP32-C6
**Note**: You'll need to create a custom symbol

**Quick Option** - Use generic header:
1. Press `A`
2. Search: `Connector_Generic:Conn_01x14` (for 14 pins)
3. Place in center of canvas
4. Right-click → `Properties`
5. Reference: `U1`
6. Value: `XIAO_ESP32-C6`
7. Label each pin (D0-D11, 3V3, GND, BAT+, BAT-) using `L` key

**Proper Option** - Create custom symbol (recommended):
1. `Tools` → `Symbol Editor`
2. `File` → `New Symbol`
3. Library: `bike_datalogger` (create new)
4. Name: `XIAO_ESP32-C6`
5. Draw rectangle, add 14 pins with proper names
6. Save and use in schematic

**Pin Labels** (for reference):
- Pin 1: D0 (GPIO0) - SPI_SCK
- Pin 2: D1 (GPIO1) - SPI_MISO
- Pin 3: D2 (GPIO2) - SPI_MOSI
- Pin 4: D3 (GPIO3) - CS_IMU_ONBOARD
- Pin 5: D4 (GPIO4) - CS_IMU_FRONT
- Pin 6: D5 (GPIO5) - CS_IMU_REAR
- Pin 7: D6 (GPIO6) - CS_SD
- Pin 8: D7 (GPIO7) - GPS_RX
- Pin 9: D8 (GPIO20) - GPS_TX
- Pin 10: D9 (GPIO21) - LED_STATUS
- Pin 11: D10 (GPIO8) - BUTTON_PWR_ON
- Pin 12: A0 (GPIO22) - PRESSURE_FRONT
- Pin 13: A2 (GPIO23) - PRESSURE_REAR
- Pin 14: 3V3 - Power output
- Pin 15: GND - Ground
- Pin 16: BAT+ - Battery positive
- Pin 17: BAT- - Battery negative (connect to GND)

#### BMI160 IMU (On-board)
1. Press `A`
2. Search: `Sensor_Motion:BMI160` (if available) OR use generic IC
3. If not in library, use `Device:C` for now and update later
4. Reference: `U2`
5. Value: `BMI160`

**BMI160 connections needed**:
- VCC
- GND
- SCK
- MISO (SDO)
- MOSI (SDI)
- CS
- INT1, INT2 (leave unconnected)

#### NEO-6M GPS Module
1. Press `A`
2. Search: `RF_GPS:NEO-6` OR `Connector_Generic:Conn_01x04`
3. Reference: `U3`
4. Value: `NEO-6M`

**GPS pins**:
- VCC
- GND
- TX (to XIAO RX)
- RX (from XIAO TX)

#### MicroSD Card Slot
1. Press `A`
2. Search: `Connector:Micro_SD_Card`
3. Reference: `J1`
4. Value: `MicroSD`

**SD pins**:
- VCC
- GND
- SCK
- MISO
- MOSI
- CS
- CD (card detect) - optional

### Step 2.3: Add Passive Components

#### Decoupling Capacitors (10µF each)
1. Press `A`
2. Search: `Device:C`
3. Place 7 capacitors total
4. Label each:
   - `C1` - XIAO 3V3 rail
   - `C2` - BMI160 on-board
   - `C3` - GPS module
   - `C4` - SD card
   - `C5` - Front connector power
   - `C6` - Rear connector power
   - `C7` - Handlebar connector
5. Set Value: `10uF`

#### Resistors
1. Press `A`
2. Search: `Device:R`
3. Add:
   - `R1` - 10kΩ - Button pull-up (D10 to 3V3)
   - `R2` - 1kΩ - Power LED current limiting
   - `R3` - 220Ω - Status LED (on handlebar connector)

#### Power LED
1. Press `A`
2. Search: `Device:LED`
3. Reference: `D1`
4. Value: `Green`
5. Connect: Anode to 3V3, Cathode to R2 to GND

### Step 2.4: Add Connectors

#### Battery Connector
1. Press `A`
2. Search: `Connector:Conn_01x02`
3. Reference: `J2`
4. Value: `JST-PH-2.0mm`
5. Pin 1: BAT+ (to XIAO BAT+)
6. Pin 2: GND

#### Front Connector (8-pin waterproof)
1. Press `A`
2. Search: `Connector_Generic:Conn_01x08`
3. Reference: `J3`
4. Value: `FRONT_CONNECTOR`
5. Pins:
   - Pin 1: 3V3
   - Pin 2: GND
   - Pin 3: SPI_SCK (D0)
   - Pin 4: SPI_MISO (D1)
   - Pin 5: SPI_MOSI (D2)
   - Pin 6: CS_IMU_FRONT (D4)
   - Pin 7: PRESSURE_FRONT (A0)
   - Pin 8: HALL_FRONT (A1 - future)

#### Rear Connector (8-pin waterproof)
1. Press `A`
2. Search: `Connector_Generic:Conn_01x08`
3. Reference: `J4`
4. Value: `REAR_CONNECTOR`
5. Pins:
   - Pin 1: 3V3
   - Pin 2: GND
   - Pin 3: SPI_SCK (D0)
   - Pin 4: SPI_MISO (D1)
   - Pin 5: SPI_MOSI (D2)
   - Pin 6: CS_IMU_REAR (D5)
   - Pin 7: PRESSURE_REAR (A2)
   - Pin 8: HALL_REAR (A3 - future)

#### Handlebar Connector (4-pin)
1. Press `A`
2. Search: `Connector_Generic:Conn_01x04`
3. Reference: `J5`
4. Value: `HANDLEBAR`
5. Pins:
   - Pin 1: GND
   - Pin 2: BUTTON_PWR_ON (D10)
   - Pin 3: BUTTON_PWR_OFF (D11 - future)
   - Pin 4: LED_STATUS (D9)

### Step 2.5: Wire the Schematic

**Wiring Tips**:
- Press `W` to start a wire
- Click start point, click end point
- Press `ESC` to cancel
- Use `L` key to add labels to nets (makes routing easier later)

**Critical Connections**:

1. **Power Distribution**:
   - XIAO 3V3 pin → All component VCC pins
   - Place `C1` close to XIAO 3V3 output
   - Each device VCC should have decoupling cap nearby

2. **SPI Bus** (shared signals):
   - XIAO D0 (SCK) → BMI160 SCK, SD SCK, Front J3-Pin3, Rear J4-Pin3
   - XIAO D1 (MISO) → BMI160 MISO, SD MISO, Front J3-Pin4, Rear J4-Pin4
   - XIAO D2 (MOSI) → BMI160 MOSI, SD MOSI, Front J3-Pin5, Rear J4-Pin5

   **Label these nets**: `SPI_SCK`, `SPI_MISO`, `SPI_MOSI`

3. **SPI Chip Selects** (individual):
   - XIAO D3 → BMI160 CS (on-board)
   - XIAO D4 → Front connector Pin 6
   - XIAO D5 → Rear connector Pin 6
   - XIAO D6 → SD card CS

4. **UART (GPS)**:
   - GPS TX → XIAO D7 (RX)
   - GPS RX → XIAO D8 (TX)

5. **GPIO**:
   - XIAO D9 → Handlebar Pin 4 (LED)
   - XIAO D10 → R1 (10kΩ to 3V3) → Handlebar Pin 2
   - XIAO A0 → Front Pin 7
   - XIAO A2 → Rear Pin 7

6. **Ground**:
   - Connect ALL GND symbols together (KiCad does this automatically)

### Step 2.6: Add Net Labels
Press `L` to add labels to important nets. This makes PCB routing MUCH easier.

**Recommended labels**:
- `SPI_SCK`
- `SPI_MISO`
- `SPI_MOSI`
- `CS_IMU_ONBOARD`
- `CS_IMU_FRONT`
- `CS_IMU_REAR`
- `CS_SD`
- `GPS_TX`
- `GPS_RX`
- `LED_STATUS`
- `BUTTON_PWR`
- `PRESSURE_FRONT`
- `PRESSURE_REAR`

### Step 2.7: Run Electrical Rules Check (ERC)
1. Click "Perform electrical rules check" icon (bug icon)
2. Click "Run ERC"
3. Fix any errors (warnings are often OK)
4. Common errors:
   - Power pins not connected
   - Unconnected pins (INT1, INT2 on BMI160 - ignore these)

---

## Phase 3: Assign Footprints (30 minutes)

### Step 3.1: Open Footprint Assignment Tool
1. `Tools` → `Assign Footprints`
2. You'll see a table with all components

### Step 3.2: Assign Footprints

#### XIAO ESP32-C6 (U1)
**Problem**: No standard footprint exists for castellated pads

**Solution**: Use SMD header temporarily
- Footprint: `Connector_PinHeader_2.54mm:PinHeader_1x14_P2.54mm_Vertical`
- **TODO**: Create custom footprint later (see Phase 7)

#### BMI160 (U2)
- Footprint: `Package_LGA:Bosch_LGA-14_3x2.5mm_P0.5mm`
- OR if using breakout board: `Module:*BMI160*` (search)
- **Recommendation**: Use breakout board for first PCB

#### NEO-6M GPS (U3)
- If module: `RF_GPS:ublox_NEO`
- If connector: `Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical`

#### MicroSD Card Slot (J1)
- Footprint: `Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5`
- OR: `Connector_Card:microSD_HC_Wuerth_693063020911`

#### JST Battery Connector (J2)
- Footprint: `Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical`

#### Front/Rear Connectors (J3, J4)
- Footprint: `Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical`
- **Note**: Replace with actual waterproof connector footprint when selected
- Suggested parts:
  - TE AMP MCP 1.5 series
  - Molex Micro-Fit 3.0

#### Handlebar Connector (J5)
- Footprint: `Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical`

#### Capacitors (C1-C7)
- All 10µF: `Capacitor_SMD:C_0805_2012Metric`
- **Specification**: X7R or X5R dielectric, 10V minimum

#### Resistors (R1, R2, R3)
- All: `Resistor_SMD:R_0805_2012Metric`

#### Power LED (D1)
- `LED_SMD:LED_0805_2012Metric`
- OR `LED_THT:LED_D3.0mm` for through-hole

### Step 3.3: Save Assignments
1. `File` → `Save` or `Ctrl+S`
2. Close Footprint Assignment window

---

## Phase 4: Generate Netlist & Open PCB Editor (10 minutes)

### Step 4.1: Generate Netlist
1. In Schematic Editor: `Tools` → `Update PCB from Schematic` (F8)
2. Click "Update PCB"
3. Dialog will open PCB editor automatically

### Step 4.2: PCB Editor Initial Setup
1. Board should open with components scattered outside board area
2. `File` → `Board Setup`

### Step 4.3: Configure Design Rules

**Under "Board Setup" → "Design Rules" → "Constraints"**:
- Minimum clearance: `0.2mm`
- Minimum track width: `0.25mm`
- Minimum annular width: `0.13mm`
- Minimum via diameter: `0.6mm`
- Minimum via drill: `0.3mm`

**Under "Net Classes"**:
- Create net class: `Power`
  - Track width: `0.5mm`
  - Clearance: `0.3mm`
- Assign `+3V3`, `GND`, `VBAT` to Power class

**Under "Board Stackup"**:
- Layers: 2 (default)
- Board thickness: `1.6mm`
- Copper weight: `1 oz (35µm)`

Save and close Board Setup.

---

## Phase 5: Board Outline & Component Placement (1-2 hours)

### Step 5.1: Draw Board Outline

**Determine board size**:
- What battery are you using? (dimensions needed)
- Suggested starting size: `60mm x 40mm` rectangle
- Add mounting holes in corners (optional)

**Draw outline**:
1. Select layer: `Edge.Cuts` (in right panel)
2. Press `Ctrl+Shift+G` for grid: set to `1mm`
3. Click "Draw Rectangle" tool (right toolbar)
4. Draw rectangle:
   - Bottom-left: `(100, 100)` - arbitrary starting point
   - Top-right: `(160, 140)` - makes 60x40mm board
5. Press `ESC` when done

**Add mounting holes** (optional):
1. Select layer: `Edge.Cuts`
2. `Place` → `Add Footprint` (or press `A`)
3. Search: `MountingHole:MountingHole_3.2mm_M3`
4. Place in four corners, ~3mm from edges

### Step 5.2: Component Placement Strategy

**Placement order** (important!):
1. XIAO ESP32-C6 (main controller)
2. Connectors (they define board edges)
3. GPS module
4. On-board BMI160
5. SD card slot
6. Decoupling capacitors
7. Resistors and LEDs

**Placement guidelines**:
- Keep USB port accessible (XIAO)
- SD card slot on accessible edge
- GPS module away from XIAO (antenna separation)
- Battery connector near XIAO BAT pads
- External connectors on edges

**Move components**:
1. Press `M` (move) or click component
2. Press `R` (rotate) while moving
3. Click to place

**Suggested layout**:
```
┌─────────────────────────────────────────┐
│                                         │
│  [GPS]              [Battery JST]       │
│                                         │
│                  [XIAO ESP32-C6]        │
│                     (USB down)          │
│                                         │
│  [BMI160]                    [SD Card]  │
│                                         │
│ [Front]                         [Rear]  │
│  J3                               J4    │
│ [Handlebar J5]                          │
└─────────────────────────────────────────┘
```

### Step 5.3: Place Decoupling Capacitors
**CRITICAL**: Place each 10µF cap within 2-3mm of the power pin it decouples

1. `C1` - next to XIAO 3V3 pin
2. `C2` - next to BMI160 VCC pin
3. `C3` - next to GPS VCC pin
4. `C4` - next to SD card VCC pin
5. `C5`, `C6`, `C7` - near front/rear/handlebar connectors

**Tip**: Press `F` while component selected to flip to bottom layer if needed

### Step 5.4: Place Remaining Components
- `R1` (10kΩ pull-up) - between XIAO D10 and 3V3
- `R2` (1kΩ LED resistor) - near power LED
- `D1` (power LED) - visible location on board edge

---

## Phase 6: Routing Traces (2-3 hours)

### Step 6.1: Auto-Route Power Planes (Optional)

**Add ground pour** (recommended):
1. Select layer: `B.Cu` (bottom copper)
2. Click "Add filled zone" (right toolbar)
3. Click corners of board to define pour area
4. Right-click → `Properties`
   - Net: `GND`
   - Clearance: `0.3mm`
   - Priority: `0`
5. Press `B` to rebuild pour

**Add 3V3 pour** (optional):
1. Select layer: `F.Cu` (top copper)
2. Same process, but:
   - Net: `+3V3`
   - Priority: `1` (lower than ground)

### Step 6.2: Manual Routing

**Route in this order**:
1. Power connections (3V3, GND to each component)
2. SPI bus (SCK, MISO, MOSI)
3. Chip select lines
4. UART (GPS)
5. Analog inputs
6. GPIO

**Routing tips**:
- Press `X` to start routing
- Click start pad, click waypoints, click end pad
- Press `V` while routing to place via
- Press `/` to switch layers
- Use `0.5mm` width for power, `0.25mm` for signals
- Keep SPI traces roughly equal length

**SPI Bus routing** (important):
- SCK, MISO, MOSI should run parallel where possible
- Keep them on same layer if possible
- Route to each SPI device (BMI160, SD, front/rear connectors)

**Critical connections**:
1. XIAO D0 → all SCK pins (BMI160, SD, J3-pin3, J4-pin3)
2. XIAO D1 → all MISO pins
3. XIAO D2 → all MOSI pins
4. Individual CS lines to each device

### Step 6.3: Add Ground Vias
**Ground stitching** improves signal integrity:
1. `Place` → `Via`
2. Place vias around board perimeter every 5-10mm
3. Ensure all vias connect top and bottom ground pours

### Step 6.4: Design Rule Check (DRC)
1. Click "Perform design rules check" icon
2. Click "Run DRC"
3. Fix any errors:
   - Clearance violations (traces too close)
   - Unconnected nets (missing routes)
   - Track width violations

---

## Phase 7: Finishing Touches (30 minutes)

### Step 7.1: Add Silkscreen Labels
1. Select layer: `F.Silkscreen`
2. Press `T` (add text)
3. Add labels:
   - "FRONT" near J3
   - "REAR" near J4
   - "BATTERY +" near JST connector
   - "SD CARD" near slot
   - "GPS" near module
   - Your name/version: "v1.0 - 2024"

### Step 7.2: Add Reference Designators
- Ensure all components show ref designators (U1, C1, etc.)
- Move them to visible locations
- Make sure they don't overlap pads

### Step 7.3: Final Checks

**Visual inspection**:
- All pads have traces connected
- No overlapping traces
- Decoupling caps close to ICs
- USB port accessible
- SD card slot accessible
- Mounting holes (if added) clear of traces

**Run DRC again**:
- Should have zero errors
- Warnings about silkscreen over pads are OK

### Step 7.4: 3D View
1. `View` → `3D Viewer`
2. Inspect board from all angles
3. Verify component placement makes sense
4. Check for clearance issues

---

## Phase 8: Generate Manufacturing Files (30 minutes)

### Step 8.1: Generate Gerber Files
1. `File` → `Plot`
2. Select layers:
   - ☑ F.Cu
   - ☑ B.Cu
   - ☑ F.Silkscreen
   - ☑ B.Silkscreen
   - ☑ F.Mask
   - ☑ B.Mask
   - ☑ Edge.Cuts
3. Output directory: `gerbers/`
4. Format: `Gerber`
5. Click "Plot"
6. Click "Generate Drill Files"
   - Format: `Excellon`
   - Click "Generate Drill File"

### Step 8.2: Generate BOM (Bill of Materials)
1. Back in Schematic Editor
2. `Tools` → `Generate BOM`
3. Select `bom_csv_grouped_by_value` plugin
4. Click "Generate"
5. Save as `bike_datalogger_BOM.csv`

### Step 8.3: Generate Assembly Files (Optional)
For PCBA service (assembled boards):
1. In PCB Editor: `File` → `Fabrication Outputs` → `Component Placement`
2. Format: CSV
3. Include:
   - Reference designator
   - X/Y position
   - Rotation
   - Layer

### Step 8.4: Verify Gerbers
1. Use online Gerber viewer: https://www.pcbway.com/project/OnlineGerberViewer.html
2. Upload all .gbr and .drl files
3. Verify:
   - Board outline correct
   - All pads present
   - Silkscreen readable
   - No shorts or errors

---

## Phase 9: Order PCBs (Manufacturer Selection)

### Recommended Manufacturers

#### Budget Option: **JLCPCB** (China)
- Cost: ~$5 for 5 boards + shipping ($15-25)
- Lead time: 2-3 weeks
- Website: https://jlcpcb.com

#### Quality Option: **OSH Park** (USA)
- Cost: ~$50 for 3 boards (free shipping)
- Lead time: 12 days
- Purple soldermask (signature)
- Website: https://oshpark.com

#### Fast Option: **PCBWay** (China)
- Cost: ~$10 for 5 boards + shipping
- Lead time: 3-7 days (express available)
- Website: https://www.pcbway.com

### Ordering Steps (JLCPCB example)
1. Go to https://jlcpcb.com
2. Click "Quote Now"
3. Upload: `bike_datalogger-gerbers.zip` (zip all gerber files)
4. Settings:
   - Layers: `2`
   - PCB Qty: `5`
   - Thickness: `1.6mm`
   - Color: `Green` (or your choice)
   - Surface Finish: `HASL` (cheap) or `ENIG` (better)
   - Remove Order Number: `Yes` (optional, costs $1.50)
5. Add to cart, checkout, pay

### What to Order
- PCBs only (you'll solder components yourself)
- OR PCBs + assembly (more expensive, requires additional files)

---

## Phase 10: After Receiving PCBs

### Visual Inspection
- Check for manufacturing defects
- Verify board dimensions
- Check silkscreen legibility

### Assembly Order
1. Solder smallest components first (resistors, capacitors)
2. Then ICs (BMI160 if bare chip)
3. Then larger components (connectors, modules)
4. Last: XIAO (use low-temp solder for castellated pads)

### Testing Plan
1. **Visual**: Check for solder bridges, cold joints
2. **Continuity**: Multimeter test GND connections
3. **Power**: Connect battery, measure 3.3V rail (no devices yet)
4. **Device-by-device**:
   - Solder XIAO only → test USB power → program "Hello World"
   - Add GPS → test UART
   - Add SD card → test SPI
   - Add BMI160 → test SPI
   - Add external connectors → test continuity

---

## Common Beginner Mistakes to Avoid

### Schematic Phase
- ❌ Forgetting power connections (VCC, GND)
- ❌ Not adding decoupling capacitors
- ❌ Swapping TX/RX on UART
- ✅ Use net labels to keep schematic clean
- ✅ Run ERC before moving to PCB

### PCB Layout Phase
- ❌ Decoupling caps far from IC power pins
- ❌ Power traces too thin (use 0.5mm minimum)
- ❌ Forgetting to connect ground plane with vias
- ❌ Routing high-speed signals too close together
- ✅ Keep SPI traces short and equal length
- ✅ Add ground pour on bottom layer

### Before Ordering
- ❌ Not running DRC
- ❌ Not checking 3D view
- ❌ Forgetting to verify Gerbers
- ✅ Have someone else review your design
- ✅ Print board outline at 1:1 scale, verify component fit

---

## Troubleshooting Guide

### "Component footprint not found"
- Download missing libraries from KiCad library manager
- Or create custom footprint (see Phase 7)

### "Clearance violation" in DRC
- Increase clearance in Board Setup
- Move traces farther apart
- Use smaller traces if space limited

### "Unconnected pad" in DRC
- Route missing traces
- Or mark as "Do not populate" (DNP) if intentional

### "Can't route all traces"
- Move components to better positions
- Use vias to switch layers
- Consider 4-layer board (more expensive)

### Gerber viewer shows weird shapes
- Check Edge.Cuts layer is correct
- Ensure filled zones are rebuilt (`B` key)

---

## Next Steps After This Guide

1. **Create custom XIAO footprint** (castellated pads)
2. **Select actual waterproof connectors** (update footprints)
3. **Consider 4-layer board** if routing is too tight
4. **Design enclosure** (3D print or buy project box)
5. **Create harness PCBs** for front/rear sensors

---

## Appendix: Quick Reference

### Keyboard Shortcuts (PCB Editor)
- `M` - Move component
- `R` - Rotate (while moving)
- `F` - Flip to other side
- `X` - Start routing trace
- `V` - Place via (while routing)
- `/` - Switch layer (while routing)
- `Delete` - Delete selected
- `ESC` - Cancel current operation
- `Ctrl+Z` - Undo
- `E` - Edit component properties

### Net Classes Summary
| Net Class | Track Width | Clearance | Nets |
|-----------|-------------|-----------|------|
| Default | 0.25mm | 0.2mm | All signals |
| Power | 0.5mm | 0.3mm | +3V3, GND, VBAT |

### Component Summary
| Ref | Component | Footprint | Value | Notes |
|-----|-----------|-----------|-------|-------|
| U1 | XIAO ESP32-C6 | Custom/Header_1x14 | - | Castellated pads |
| U2 | BMI160 | LGA-14 or breakout | - | On-board IMU |
| U3 | NEO-6M | Module | - | GPS |
| J1 | MicroSD | microSD_HC | - | Card slot |
| J2 | Battery | JST-PH-2.0mm | - | 2-pin |
| J3/J4 | External | Header_1x08 | - | Front/Rear |
| J5 | Handlebar | Header_1x04 | - | Buttons/LED |
| C1-C7 | Capacitor | 0805 | 10µF | X7R/X5R |
| R1 | Resistor | 0805 | 10kΩ | Pull-up |
| R2 | Resistor | 0805 | 1kΩ | LED |
| R3 | Resistor | 0805 | 220Ω | LED |
| D1 | LED | 0805 | Green | Power indicator |

### Pin Mapping Quick Reference
```
XIAO ESP32-C6 Pin Assignments:
D0  → SPI_SCK (shared)
D1  → SPI_MISO (shared)
D2  → SPI_MOSI (shared)
D3  → CS_IMU_ONBOARD
D4  → CS_IMU_FRONT
D5  → CS_IMU_REAR
D6  → CS_SD
D7  → GPS_RX (from GPS TX)
D8  → GPS_TX (to GPS RX)
D9  → LED_STATUS (handlebar)
D10 → BUTTON_PWR (with 10kΩ pull-up)
A0  → PRESSURE_FRONT (analog)
A2  → PRESSURE_REAR (analog)
3V3 → Power distribution (to all devices)
GND → Ground plane
BAT+→ JST battery connector
```

---

## Estimated Costs

### PCB Manufacturing
- JLCPCB: $5 + $15-25 shipping = **$20-30 total**
- OSH Park: **$50** (free shipping)

### Components (approximate)
- XIAO ESP32-C6: $7
- BMI160 breakout (x3): $15
- NEO-6M GPS: $10
- MicroSD slot: $2
- JST connector: $1
- Headers/connectors: $5
- Capacitors/resistors: $3
- Misc (LED, antenna connector): $5
- **Total components: ~$50**

### Tools Needed
- Soldering iron + solder
- Multimeter
- Flux, tweezers, wire
- (Optional) Hot air station for SMD rework

**Grand Total: ~$100-150** for first prototype batch

---

## Resources

### KiCad Learning
- Official KiCad docs: https://docs.kicad.org/
- Getting Started in KiCad (video): https://www.youtube.com/watch?v=vaCVh2SAZY4
- DigiKey KiCad tutorial series: https://www.digikey.com/en/resources/design-tools/kicad

### Component Datasheets
- BMI160: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf
- NEO-6M: https://www.u-blox.com/en/docs/UBX-13003221
- ESP32-C6: https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf

### PCB Design Guidelines
- IPC-2221: Generic PCB design standard
- Seeed Fusion PCB specs: https://www.seeedstudio.com/fusion_pcb.html

---

## Conclusion

You now have a complete workflow to design your mountain bike datalogger PCB!

**Timeline estimate**:
- First-time: 6-10 hours total
- With experience: 3-4 hours

**Remember**:
- Take your time in schematic phase (errors are costly to fix later)
- Run DRC frequently
- Verify Gerbers before ordering
- Start simple, iterate to improve

Good luck with your first PCB! 🚴‍♂️

---

*Created: 2024*
*For: Mountain Bike Datalogger v2*
*Board: XIAO ESP32-C6*
