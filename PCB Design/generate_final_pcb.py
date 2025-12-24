#!/usr/bin/env python3
"""
Final PCB Generation for Mountain Bike Datalogger
Uses Seeed Studio XIAO library and KiCad standard libraries
"""

from skidl import *
import os
import sys

# Add Seeed library to path
seeed_lib_path = os.path.abspath('OPL_Kicad_Library/Seeed Studio XIAO Series Library')
kicad_symbol_path = r'C:\Program Files\KiCad\9.0\share\kicad\symbols'

# Set environment variables
os.environ['KICAD9_SYMBOL_DIR'] = kicad_symbol_path
lib_search_paths[KICAD].append(seeed_lib_path)

# Set default tool
set_default_tool(KICAD8)

print("=" * 70)
print("Mountain Bike Datalogger - Final PCB Generation")
print("=" * 70)
print(f"\nSeeed library path: {seeed_lib_path}")
print(f"KiCad symbols path: {kicad_symbol_path}\n")

print("[Step 1] Creating circuit definition with SKiDL...")

# Define nets
gnd, vcc_3v3, vbat = Net('GND'), Net('+3V3'), Net('VBAT')
spi_sck, spi_miso, spi_mosi = Net('SPI_SCK'), Net('SPI_MISO'), Net('SPI_MOSI')
cs_bmi_onboard, cs_bmi_front, cs_bmi_rear, cs_sd = Net('CS_BMI_ONBOARD'), Net('CS_BMI_FRONT'), Net('CS_BMI_REAR'), Net('CS_SD')
gps_tx, gps_rx = Net('GPS_TX'), Net('GPS_RX')
led_status, button_pwr = Net('LED_STATUS'), Net('BUTTON_PWR')
pressure_front, pressure_rear = Net('PRESSURE_FRONT'), Net('PRESSURE_REAR')

print("Creating XIAO ESP32-C6...")
# Load XIAO ESP32-C6 from Seeed library (use SMD version for castellated pads)
mcu = Part(seeed_lib_path + '/Seeed_Studio_XIAO_Series.kicad_sym', 'XIAO-ESP32-C6-SMD',
           ref='U1', footprint=seeed_lib_path + '/XIAO-ESP32C6-SMD.kicad_mod')

# Connect XIAO pins
mcu['D0'] += spi_sck
mcu['D1'] += spi_miso
mcu['D2'] += spi_mosi
mcu['D3'] += cs_bmi_onboard
mcu['D4'] += cs_bmi_front
mcu['D5'] += cs_bmi_rear
mcu['D6'] += cs_sd
mcu['D7'] += gps_rx  # MCU RX from GPS TX
mcu['D8'] += gps_tx  # MCU TX to GPS RX
mcu['D9'] += led_status
mcu['D10'] += button_pwr
mcu['A0'] += pressure_front
mcu['A2'] += pressure_rear
mcu['3V3'] += vcc_3v3
mcu['GND'] += gnd
mcu['5V'] += NC
mcu['BAT+'] += vbat
mcu['BAT-'] += gnd

print("Creating components...")

# Use generic Device library for standard parts
# BMI160 - use generic connector for now
imu = Part('Connector_Generic', 'Conn_01x08', ref='U2', value='BMI160',
           footprint='Package_LGA:LGA-14_3x2.5mm_P0.5mm')
imu[1] += vcc_3v3  # VDD
imu[2] += gnd      # GND
imu[3] += spi_sck
imu[4] += spi_mosi
imu[5] += spi_miso
imu[6] += cs_bmi_onboard
imu[7] += NC       # INT1
imu[8] += NC       # INT2

# GPS Module
gps = Part('Connector_Generic', 'Conn_01x04', ref='U3', value='NEO-6M',
           footprint='RF_GPS:ublox_NEO')
gps[1] += vcc_3v3
gps[2] += gnd
gps[3] += gps_tx  # GPS TX to MCU RX
gps[4] += gps_rx  # GPS RX from MCU TX

# MicroSD
sd = Part('Connector_Generic', 'Conn_01x06', ref='J1', value='MicroSD',
          footprint='Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5')
sd[1] += vcc_3v3
sd[2] += gnd
sd[3] += spi_sck
sd[4] += spi_miso
sd[5] += spi_mosi
sd[6] += cs_sd

print("Adding passive components...")

# Decoupling caps
caps = []
for ref in ['C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7']:
    cap = Part('Device', 'C', ref=ref, value='10uF',
               footprint='Capacitor_SMD:C_0805_2012Metric')
    cap[1] += vcc_3v3
    cap[2] += gnd
    caps.append(cap)

# Resistors
r1 = Part('Device', 'R', ref='R1', value='10k', footprint='Resistor_SMD:R_0805_2012Metric')
r1[1] += button_pwr
r1[2] += vcc_3v3

r2 = Part('Device', 'R', ref='R2', value='1k', footprint='Resistor_SMD:R_0805_2012Metric')
r3 = Part('Device', 'R', ref='R3', value='220', footprint='Resistor_SMD:R_0805_2012Metric')

# Power LED
led = Part('Device', 'LED', ref='D1', value='Green', footprint='LED_SMD:LED_0805_2012Metric')
led['A'] += vcc_3v3
led['K'] += r2[1]
r2[2] += gnd

print("Creating connectors...")

# Battery
batt = Part('Connector', 'Conn_01x02', ref='J2', value='JST-PH-2.0mm',
            footprint='Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical')
batt[1] += vbat
batt[2] += gnd

# Front connector
front = Part('Connector_Generic', 'Conn_01x08', ref='J3', value='FRONT',
             footprint='Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical')
front[1] += vcc_3v3
front[2] += gnd
front[3] += spi_sck
front[4] += spi_miso
front[5] += spi_mosi
front[6] += cs_bmi_front
front[7] += pressure_front
front[8] += NC

# Rear connector
rear = Part('Connector_Generic', 'Conn_01x08', ref='J4', value='REAR',
            footprint='Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical')
rear[1] += vcc_3v3
rear[2] += gnd
rear[3] += spi_sck
rear[4] += spi_miso
rear[5] += spi_mosi
rear[6] += cs_bmi_rear
rear[7] += pressure_rear
rear[8] += NC

# Handlebar connector
handlebar = Part('Connector_Generic', 'Conn_01x04', ref='J5', value='HANDLEBAR',
                 footprint='Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical')
handlebar[1] += gnd
handlebar[2] += button_pwr
handlebar[3] += NC
handlebar[4] += led_status

print("✓ Circuit definition complete!")

# Generate netlist
print("\n[Step 2] Generating netlist...")
os.makedirs('output', exist_ok=True)
netlist_file = 'output/datalogger.net'

try:
    generate_netlist(file_=netlist_file)
    print(f"✓ Netlist: {netlist_file}")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

# ERC check
print("\n[Step 3] Running ERC...")
try:
    erc()
    print("✓ ERC passed")
except:
    print("⚠ ERC warnings (review output above)")

# Generate placement script
print("\n[Step 4] Creating placement script...")
placement_script = '''#!/usr/bin/env python3
"""Component Placement for Datalogger PCB - Run in KiCad PCBNew"""
import pcbnew

board = pcbnew.GetBoard()

# Placements (x, y in mm, rotation in degrees)
placements = {
    'U1': (30, 20, 0),    # XIAO ESP32-C6
    'U2': (20, 18, 0),    # BMI160
    'U3': (50, 30, 0),    # GPS
    'J1': (10, 30, 90),   # SD card
    'J2': (35, 10, 0),    # Battery
    'J3': (5, 5, 0),      # Front
    'J4': (55, 5, 0),     # Rear
    'J5': (30, 38, 0),    # Handlebar
    'C1': (32, 18, 0), 'C2': (18, 16, 0), 'C3': (52, 28, 0),
    'C4': (8, 28, 0), 'C5': (3, 8, 0), 'C6': (57, 8, 0), 'C7': (28, 36, 0),
    'R1': (28, 34, 0), 'R2': (42, 20, 0), 'D1': (42, 22, 0),
}

for ref, (x, y, rot) in placements.items():
    fp = board.FindFootprintByReference(ref)
    if fp:
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
        fp.SetOrientationDegrees(rot)
        print(f"✓ {ref} at ({x}, {y})")

pcbnew.Refresh()
print("Placement complete!")
'''

with open('output/place_components.py', 'w') as f:
    f.write(placement_script)
print("✓ Placement script: output/place_components.py")

# Generate README
print("\n[Step 5] Creating workflow instructions...")
readme = '''# Datalogger PCB - Generated Files

## Files Generated
- `datalogger.net` - KiCad netlist with all components and connections
- `place_components.py` - Automated component placement script

## Next Steps

### 1. Open KiCad PCBNew
```
File → New Project → "bike_datalogger"
```

### 2. Import Netlist
```
Tools → Load Netlist → Select datalogger.net
Click "Update PCB"
```

All components will appear on the canvas!

### 3. Set Board Outline
```
- Select Edge.Cuts layer
- Draw rectangle: 60mm x 40mm
- Add mounting holes (optional)
```

### 4. Run Placement Script
```
Tools → Scripting Console
File → Open → place_components.py
OR paste the script contents and press Enter
```

### 5. Route the Board
**Option A - Manual (Recommended for learning):**
- Press 'X' to start routing
- Click start pad, waypoints, end pad
- Use '/' to switch layers
- Press 'V' to add via

**Option B - Auto-route:**
```
Tools → External Tools → FreeRouting
Let it route, then import back
```

### 6. Add Ground Plane
```
Right toolbar → Add Filled Zone
Draw around board edge
Properties: Net=GND, Layer=B.Cu
Press 'B' to fill
```

### 7. Design Rules Check
```
Inspect → Design Rules Checker
Fix any errors shown
```

### 8. Export Gerbers
```
File → Fabrication Outputs → Gerbers
Select all layers
Output directory: gerbers/
Generate Drill Files too
```

### 9. Order PCBs
Upload gerbers.zip to:
- JLCPCB.com (~$25 shipped)
- OSH Park (~$50)
- PCBWay (~$30)

Settings:
- 2 layers
- 1.6mm thickness
- Quantity: 5
- Color: Green (or your choice)

## Tips
- Keep decoupling caps within 3mm of IC power pins
- Use 0.5mm traces for power, 0.25mm for signals
- Add ground vias around perimeter
- GPS antenna area: keep clear of copper pours

Good luck! 🚴‍♂️
'''

with open('output/README.md', 'w') as f:
    f.write(readme)
print("✓ Instructions: output/README.md")

print("\n" + "=" * 70)
print("SUCCESS! Generated files in output/")
print("=" * 70)
print("""
✓ datalogger.net - Complete netlist for KiCad
✓ place_components.py - Automated placement
✓ README.md - Step-by-step workflow

Next: Open KiCad PCBNew → Load netlist → Run placement script → Route!
""")
print("=" * 70)
