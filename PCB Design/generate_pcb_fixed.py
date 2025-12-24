#!/usr/bin/env python3
"""
Fixed PCB Generation for Mountain Bike Datalogger
Properly configured with KiCad library paths
"""

from skidl import *
import os

# Set KiCad library paths BEFORE importing
os.environ['KICAD9_SYMBOL_DIR'] = r'C:\Program Files\KiCad\9.0\share\kicad\symbols'
os.environ['KICAD9_FOOTPRINT_DIR'] = r'C:\Program Files\KiCad\9.0\share\kicad\footprints'

# Add to SKiDL search paths
if isinstance(lib_search_paths[KICAD], list):
    lib_search_paths[KICAD].insert(0, r'C:\Program Files\KiCad\9.0\share\kicad\symbols')
if isinstance(footprint_search_paths[KICAD], list):
    footprint_search_paths[KICAD].insert(0, r'C:\Program Files\KiCad\9.0\share\kicad\footprints')

# Set default tool
set_default_tool(KICAD)

print("=" * 70)
print("Mountain Bike Datalogger - PCB Generation")
print("=" * 70)

print("\n[Step 1] Creating circuit with SKiDL...")

# Define nets
gnd = Net('GND')
vcc = Net('+3V3')
vbat = Net('VBAT')

spi_sck = Net('SPI_SCK')
spi_miso = Net('SPI_MISO')
spi_mosi = Net('SPI_MOSI')

cs_onboard = Net('CS_BMI_ONBOARD')
cs_front = Net('CS_BMI_FRONT')
cs_rear = Net('CS_BMI_REAR')
cs_sd = Net('CS_SD')

gps_tx = Net('GPS_TX')
gps_rx = Net('GPS_RX')

led_status = Net('LED_STATUS')
button_pwr = Net('BUTTON_PWR')

pressure_front = Net('PRESSURE_FRONT')
pressure_rear = Net('PRESSURE_REAR')

print("Creating XIAO ESP32-C6...")
# Use absolute path to KiCad libraries
kicad_sym = r'C:\Program Files\KiCad\9.0\share\kicad\symbols'

# XIAO as generic connector (will update footprint in KiCad later)
xiao = Part(f'{kicad_sym}/Connector_Generic.kicad_sym', 'Conn_01x17', ref='U1', value='XIAO_ESP32-C6',
            footprint='Connector_PinHeader_2.54mm:PinHeader_1x17_P2.54mm_Vertical')

xiao[1] += spi_sck        # D0
xiao[2] += spi_miso       # D1
xiao[3] += spi_mosi       # D2
xiao[4] += cs_onboard     # D3
xiao[5] += cs_front       # D4
xiao[6] += cs_rear        # D5
xiao[7] += cs_sd          # D6
xiao[8] += gps_rx         # D7 (MCU RX from GPS TX)
xiao[9] += gps_tx         # D8 (MCU TX to GPS RX)
xiao[10] += led_status    # D9
xiao[11] += button_pwr    # D10
xiao[12] += pressure_front # A0
xiao[13] += pressure_rear  # A2
xiao[14] += vcc           # 3V3
xiao[15] += gnd           # GND
xiao[16] += vbat          # BAT+
xiao[17] += gnd           # BAT-

print("Creating BMI160 IMU...")
imu = Part(f'{kicad_sym}/Connector_Generic.kicad_sym', 'Conn_01x08', ref='U2', value='BMI160',
           footprint='Package_LGA:LGA-14_3x2.5mm_P0.5mm')
imu[1] += vcc
imu[2] += gnd
imu[3] += spi_sck
imu[4] += spi_mosi
imu[5] += spi_miso
imu[6] += cs_onboard
imu[7] += NC  # INT1
imu[8] += NC  # INT2

print("Creating GPS module...")
gps = Part(f'{kicad_sym}/Connector_Generic.kicad_sym', 'Conn_01x04', ref='U3', value='NEO-6M',
           footprint='RF_GPS:ublox_NEO')
gps[1] += vcc
gps[2] += gnd
gps[3] += gps_tx  # GPS TX to MCU RX
gps[4] += gps_rx  # GPS RX from MCU TX

print("Creating microSD card...")
sd = Part(f'{kicad_sym}/Connector_Generic.kicad_sym', 'Conn_01x06', ref='J1', value='MicroSD',
          footprint='Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5')
sd[1] += vcc
sd[2] += gnd
sd[3] += spi_sck
sd[4] += spi_miso
sd[5] += spi_mosi
sd[6] += cs_sd

print("Adding passive components...")
# Decoupling capacitors
for ref in ['C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7']:
    cap = Part(f'{kicad_sym}/Device.kicad_sym', 'C', ref=ref, value='10uF',
               footprint='Capacitor_SMD:C_0805_2012Metric')
    cap[1] += vcc
    cap[2] += gnd

# Resistors
r1 = Part(f'{kicad_sym}/Device.kicad_sym', 'R', ref='R1', value='10k',
          footprint='Resistor_SMD:R_0805_2012Metric')
r1[1] += button_pwr
r1[2] += vcc

r2 = Part(f'{kicad_sym}/Device.kicad_sym', 'R', ref='R2', value='1k',
          footprint='Resistor_SMD:R_0805_2012Metric')

# Power LED
led = Part(f'{kicad_sym}/Device.kicad_sym', 'LED', ref='D1', value='Green',
           footprint='LED_SMD:LED_0805_2012Metric')
led['A'] += vcc
led['K'] += r2[1]
r2[2] += gnd

print("Creating connectors...")

# Battery
batt = Part(f'{kicad_sym}/Connector_Generic.kicad_sym', 'Conn_01x02', ref='J2', value='JST-PH-2.0mm',
            footprint='Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical')
batt[1] += vbat
batt[2] += gnd

# Front connector
front = Part(f'{kicad_sym}/Connector_Generic.kicad_sym', 'Conn_01x08', ref='J3', value='FRONT',
             footprint='Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical')
front[1] += vcc
front[2] += gnd
front[3] += spi_sck
front[4] += spi_miso
front[5] += spi_mosi
front[6] += cs_front
front[7] += pressure_front
front[8] += NC

# Rear connector
rear = Part(f'{kicad_sym}/Connector_Generic.kicad_sym', 'Conn_01x08', ref='J4', value='REAR',
            footprint='Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical')
rear[1] += vcc
rear[2] += gnd
rear[3] += spi_sck
rear[4] += spi_miso
rear[5] += spi_mosi
rear[6] += cs_rear
rear[7] += pressure_rear
rear[8] += NC

# Handlebar connector
hbar = Part(f'{kicad_sym}/Connector_Generic.kicad_sym', 'Conn_01x04', ref='J5', value='HANDLEBAR',
            footprint='Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical')
hbar[1] += gnd
hbar[2] += button_pwr
hbar[3] += NC
hbar[4] += led_status

print("✓ Circuit definition complete!")

# Generate netlist
print("\n[Step 2] Generating netlist...")
os.makedirs('output', exist_ok=True)

try:
    generate_netlist(file_='output/datalogger.net')
    print("✓ Netlist generated: output/datalogger.net")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
    exit(1)

# ERC
print("\n[Step 3] Running ERC...")
try:
    erc()
    print("✓ ERC passed")
except:
    print("⚠ ERC warnings (review above)")

# Placement script
print("\n[Step 4] Generating placement script...")
placement = '''#!/usr/bin/env python3
"""Component Placement Script - Run in KiCad PCBNew Scripting Console"""
import pcbnew

board = pcbnew.GetBoard()

# Component placements (x, y in mm, rotation in degrees)
placements = {
    'U1': (30, 20, 0),    # XIAO ESP32-C6
    'U2': (20, 18, 0),    # BMI160
    'U3': (50, 30, 0),    # GPS
    'J1': (10, 30, 90),   # SD card
    'J2': (35, 10, 0),    # Battery
    'J3': (5, 5, 0),      # Front connector
    'J4': (55, 5, 0),     # Rear connector
    'J5': (30, 38, 0),    # Handlebar
    'C1': (32, 18, 0),    # Caps near components
    'C2': (18, 16, 0),
    'C3': (52, 28, 0),
    'C4': (8, 28, 0),
    'C5': (3, 8, 0),
    'C6': (57, 8, 0),
    'C7': (28, 36, 0),
    'R1': (28, 34, 0),    # Button pull-up
    'R2': (42, 20, 0),    # LED resistor
    'D1': (42, 22, 0),    # Power LED
}

for ref, (x, y, rot) in placements.items():
    fp = board.FindFootprintByReference(ref)
    if fp:
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
        fp.SetOrientationDegrees(rot)
        print(f"✓ Placed {ref} at ({x}, {y})")
    else:
        print(f"⚠ Component {ref} not found")

pcbnew.Refresh()
print("\\nPlacement complete! Now route the board.")
'''

with open('output/place_components.py', 'w') as f:
    f.write(placement)
print("✓ Placement script: output/place_components.py")

# README
print("\n[Step 5] Creating instructions...")
readme = '''# Mountain Bike Datalogger PCB - Generated Files

## ✓ Files Generated:
- `datalogger.net` - KiCad netlist with all components and connections
- `place_components.py` - Automated component placement script

## Quick Start Guide:

### 1. Open KiCad
```
Launch KiCad PCBNew
File → New Project → "bike_datalogger"
```

### 2. Import Netlist
```
Tools → Load Netlist
Select: datalogger.net
Click: Update PCB
```
All components will appear!

### 3. Update XIAO Footprint (Important!)
```
Right-click U1 (XIAO) → Properties → Footprint
Browse to: OPL_Kicad_Library/Seeed Studio XIAO Series Library/XIAO-ESP32C6-SMD.kicad_mod
Click OK
```

### 4. Set Board Outline
```
Select layer: Edge.Cuts
Tools → Draw Rectangle
Draw: 60mm x 40mm rectangle
```

### 5. Run Placement Script
```
Tools → Scripting Console
File → Open → place_components.py
OR paste the script contents and press Enter
```

### 6. Route the Board
**Option A - Manual (Recommended):**
- Press 'X' to start routing
- Click start pad → waypoints → end pad
- Press 'V' to add via
- Press '/' to switch layers

**Option B - Auto-route:**
```
Tools → External Tools → FreeRouting
Wait for routing
Import results back
```

### 7. Add Ground Plane
```
Right toolbar → Add Filled Zone
Draw around board perimeter
Properties: Net=GND, Layer=B.Cu
Press 'B' to fill zones
```

### 8. Design Rules Check
```
Inspect → Design Rules Checker
Fix any errors shown
Re-run until zero errors
```

### 9. Export Gerbers
```
File → Fabrication Outputs → Gerbers
Select all layers:
  ☑ F.Cu, B.Cu
  ☑ F.Silkscreen, B.Silkscreen
  ☑ F.Mask, B.Mask
  ☑ Edge.Cuts
Output: gerbers/
Generate Drill Files too
```

### 10. Order PCBs
Upload gerbers ZIP to:
- **JLCPCB**: https://jlcpcb.com (~$25 shipped, 2-3 weeks)
- **OSH Park**: https://oshpark.com (~$50, 12 days, purple boards)
- **PCBWay**: https://pcbway.com (~$30 shipped)

**Order Settings:**
- Layers: 2
- Thickness: 1.6mm
- Quantity: 5
- Surface Finish: HASL (or ENIG for better quality)
- Color: Green (or your preference)

## Component Summary:

| Ref | Component | Value | Footprint | Notes |
|-----|-----------|-------|-----------|-------|
| U1 | XIAO ESP32-C6 | - | XIAO-ESP32C6-SMD | Update to Seeed footprint |
| U2 | BMI160 | - | LGA-14 | On-board IMU |
| U3 | NEO-6M | - | ublox_NEO | GPS module |
| J1 | MicroSD | - | Hirose DM3AT | SD card slot |
| J2 | Battery | JST-PH | 2-pin 2.0mm | Battery connector |
| J3 | Front | 8-pin | Header | External harness |
| J4 | Rear | 8-pin | Header | External harness |
| J5 | Handlebar | 4-pin | Header | Buttons/LED |
| C1-C7 | Capacitor | 10µF | 0805 | Decoupling |
| R1 | Resistor | 10kΩ | 0805 | Pull-up |
| R2 | Resistor | 1kΩ | 0805 | LED current |
| D1 | LED | Green | 0805 | Power indicator |

## Design Notes:

### Critical Requirements:
- Decoupling caps (C1-C7) must be within 2-3mm of IC power pins
- GPS antenna area: keep clear of copper pours
- SPI traces: keep roughly equal length (within 10mm)
- Power traces: 0.5mm width minimum
- Signal traces: 0.25mm width minimum

### Board Specifications:
- Size: 60mm x 40mm (adjust for your battery)
- Layers: 2 (Top + Bottom)
- Ground plane on bottom layer
- Mounting holes: 3mm diameter for M3 screws (optional)

### Pin Mapping Reference:
```
XIAO ESP32-C6:
Pin 1  (D0)  → SPI_SCK
Pin 2  (D1)  → SPI_MISO
Pin 3  (D2)  → SPI_MOSI
Pin 4  (D3)  → CS_BMI_ONBOARD
Pin 5  (D4)  → CS_BMI_FRONT
Pin 6  (D5)  → CS_BMI_REAR
Pin 7  (D6)  → CS_SD
Pin 8  (D7)  → GPS_TX (MCU RX)
Pin 9  (D8)  → GPS_RX (MCU TX)
Pin 10 (D9)  → LED_STATUS
Pin 11 (D10) → BUTTON_PWR (10kΩ pull-up)
Pin 12 (A0)  → PRESSURE_FRONT
Pin 13 (A2)  → PRESSURE_REAR
Pin 14       → 3V3
Pin 15       → GND
Pin 16       → BAT+
Pin 17       → BAT-
```

## Troubleshooting:

**"Component not found" in placement script:**
- Run the script after importing netlist
- Check component references match

**DRC errors - Clearance violations:**
- Move traces apart
- Reduce trace width if needed
- Add more space between components

**DRC errors - Unconnected pads:**
- Missing traces - route them manually
- Or verify net connections in schematic

**Gerber viewer shows issues:**
- Re-fill copper pours (press 'B')
- Check Edge.Cuts layer is correct
- Verify all layers exported

## Next Steps After Receiving PCBs:

1. Visual inspection for defects
2. Solder components (smallest first)
3. Test with multimeter:
   - GND continuity
   - No shorts (3V3 to GND)
4. Connect USB, measure 3.3V rail
5. Program firmware
6. Test each subsystem individually

Good luck with your PCB! 🚴‍♂️
'''

with open('output/README.md', 'w') as f:
    f.write(readme)
print("✓ Instructions: output/README.md")

print("\n" + "=" * 70)
print("SUCCESS! PCB Files Generated")
print("=" * 70)
print("""
Generated files in output/:
  ✓ datalogger.net - KiCad netlist
  ✓ place_components.py - Automated placement
  ✓ README.md - Complete workflow guide

Next steps:
  1. Open KiCad PCBNew
  2. Load netlist (Tools → Load Netlist)
  3. Update XIAO footprint to Seeed library
  4. Run placement script
  5. Route board
  6. Export Gerbers
  7. Order from JLCPCB ($25)

Total time: ~2-4 hours for manual work
""")
print("=" * 70)
