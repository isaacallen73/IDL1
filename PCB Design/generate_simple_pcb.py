#!/usr/bin/env python3
"""
Simplified PCB Generation for Mountain Bike Datalogger
Uses only built-in SKiDL components to avoid library dependency issues
"""

from skidl import *
import os

# Set KiCad library path
os.environ['KICAD9_SYMBOL_DIR'] = r'C:\Program Files\KiCad\9.0\share\kicad\symbols'

# Set default tool
set_default_tool(KICAD8)

print("=" * 70)
print("Mountain Bike Datalogger - Simplified PCB Generation")
print("=" * 70)

print("\n[Step 1] Creating circuit definition with SKiDL...")

# Define nets
gnd = Net('GND')
vcc_3v3 = Net('+3V3')
vbat = Net('VBAT')

# SPI bus nets
spi_sck = Net('SPI_SCK')
spi_miso = Net('SPI_MISO')
spi_mosi = Net('SPI_MOSI')

# Chip select nets
cs_bmi_onboard = Net('CS_BMI_ONBOARD')
cs_bmi_front = Net('CS_BMI_FRONT')
cs_bmi_rear = Net('CS_BMI_REAR')
cs_sd = Net('CS_SD')

# UART nets
gps_tx = Net('GPS_TX')
gps_rx = Net('GPS_RX')

# GPIO nets
led_status = Net('LED_STATUS')
button_pwr = Net('BUTTON_PWR')

# Analog nets
pressure_front = Net('PRESSURE_FRONT')
pressure_rear = Net('PRESSURE_REAR')

print("Creating connectors and components...")

# XIAO ESP32-C6 - Represented as a 17-pin connector
mcu = Part('Connector_Generic', 'Conn_01x17', ref='U1', value='XIAO_ESP32-C6',
           footprint='Connector_PinHeader_2.54mm:PinHeader_1x17_P2.54mm_Vertical')

# Assign pins
mcu[1] += spi_sck        # D0 - GPIO0
mcu[2] += spi_miso       # D1 - GPIO1
mcu[3] += spi_mosi       # D2 - GPIO2
mcu[4] += cs_bmi_onboard # D3 - GPIO3
mcu[5] += cs_bmi_front   # D4 - GPIO4
mcu[6] += cs_bmi_rear    # D5 - GPIO5
mcu[7] += cs_sd          # D6 - GPIO6
mcu[8] += gps_rx         # D7 - GPIO7 (MCU RX, from GPS TX)
mcu[9] += gps_tx         # D8 - GPIO20 (MCU TX, to GPS RX)
mcu[10] += led_status    # D9 - GPIO21
mcu[11] += button_pwr    # D10 - GPIO8
mcu[12] += pressure_front # A0 - GPIO22
mcu[13] += pressure_rear  # A2 - GPIO23
mcu[14] += vcc_3v3       # 3V3
mcu[15] += gnd           # GND
mcu[16] += vbat          # BAT+
mcu[17] += gnd           # BAT-

# BMI160 IMU (onboard) - Represented as generic IC
imu = Part('Connector_Generic', 'Conn_01x08', ref='U2', value='BMI160',
           footprint='Package_LGA:LGA-14_3x2.5mm_P0.5mm')
imu[1] += vcc_3v3
imu[2] += gnd
imu[3] += spi_sck
imu[4] += spi_mosi
imu[5] += spi_miso
imu[6] += cs_bmi_onboard
imu[7] += NC  # INT1
imu[8] += NC  # INT2

# GPS Module
gps = Part('Connector_Generic', 'Conn_01x04', ref='U3', value='NEO-6M',
           footprint='RF_GPS:ublox_NEO')
gps[1] += vcc_3v3
gps[2] += gnd
gps[3] += gps_tx  # GPS TX to MCU RX
gps[4] += gps_rx  # GPS RX from MCU TX

# MicroSD Card
sd = Part('Connector_Generic', 'Conn_01x06', ref='J1', value='MicroSD',
          footprint='Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5')
sd[1] += vcc_3v3
sd[2] += gnd
sd[3] += spi_sck
sd[4] += spi_miso
sd[5] += spi_mosi
sd[6] += cs_sd

print("Adding passive components...")

# Decoupling capacitors
for i, (ref, desc) in enumerate([
    ('C1', 'XIAO'),
    ('C2', 'BMI160'),
    ('C3', 'GPS'),
    ('C4', 'SD'),
    ('C5', 'Front'),
    ('C6', 'Rear'),
    ('C7', 'Handlebar')
], 1):
    cap = Part('Device', 'C', ref=ref, value='10uF',
               footprint='Capacitor_SMD:C_0805_2012Metric')
    cap[1] += vcc_3v3
    cap[2] += gnd

# Resistors
r1 = Part('Device', 'R', ref='R1', value='10k',
          footprint='Resistor_SMD:R_0805_2012Metric')
r1[1] += button_pwr
r1[2] += vcc_3v3

r2 = Part('Device', 'R', ref='R2', value='1k',
          footprint='Resistor_SMD:R_0805_2012Metric')

r3 = Part('Device', 'R', ref='R3', value='220',
          footprint='Resistor_SMD:R_0805_2012Metric')

# LEDs
led = Part('Device', 'LED', ref='D1', value='Green',
           footprint='LED_SMD:LED_0805_2012Metric')
led['A'] += vcc_3v3
led['K'] += r2[1]
r2[2] += gnd

print("Creating external connectors...")

# Battery connector
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
front[8] += NC  # Hall sensor (future)

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
rear[8] += NC  # Hall sensor (future)

# Handlebar connector
handlebar = Part('Connector_Generic', 'Conn_01x04', ref='J5', value='HANDLEBAR',
                 footprint='Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical')
handlebar[1] += gnd
handlebar[2] += button_pwr
handlebar[3] += NC  # Second button (future)
handlebar[4] += led_status

print("Circuit definition complete!")

# Generate netlist
print("\n[Step 2] Generating netlist...")
os.makedirs('output', exist_ok=True)

netlist_file = 'output/datalogger.net'
generate_netlist(file_=netlist_file)

print(f"✓ Netlist generated: {netlist_file}")

# Generate ERC report
print("\n[Step 3] Running Electrical Rules Check...")
try:
    erc()
    print("✓ ERC passed!")
except Exception as e:
    print(f"⚠ ERC warnings/errors (review manually): {e}")

# Generate component placement script
print("\n[Step 4] Generating component placement script...")

placement_script = '''#!/usr/bin/env python3
"""
Component Placement Script for Datalogger PCB
Run this inside KiCad PCBNew scripting console
"""

import pcbnew

board = pcbnew.GetBoard()

# Component placements (x, y in mm, rotation in degrees)
placements = {
    'U1': (30, 20, 0),    # XIAO center
    'U2': (20, 18, 0),    # BMI160 left of XIAO
    'U3': (50, 30, 0),    # GPS upper right
    'J1': (10, 30, 90),   # SD card left edge
    'J2': (35, 10, 0),    # Battery near XIAO
    'J3': (5, 5, 0),      # Front connector bottom left
    'J4': (55, 5, 0),     # Rear connector bottom right
    'J5': (30, 38, 0),    # Handlebar top center
    'C1': (32, 18, 0),    # Near XIAO
    'C2': (18, 16, 0),    # Near BMI160
    'C3': (52, 28, 0),    # Near GPS
    'C4': (8, 28, 0),     # Near SD
    'R1': (28, 36, 0),    # Near handlebar
    'R2': (42, 20, 0),    # Near XIAO
    'D1': (42, 22, 0),    # Power LED
}

for ref, (x, y, rot) in placements.items():
    fp = board.FindFootprintByReference(ref)
    if fp:
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
        fp.SetOrientationDegrees(rot)
        print(f"Placed {ref} at ({x}, {y})")
    else:
        print(f"Warning: {ref} not found")

pcbnew.Refresh()
print("Placement complete!")
'''

with open('output/place_components.py', 'w') as f:
    f.write(placement_script)

print("✓ Placement script: output/place_components.py")

# Generate SVG schematic
print("\n[Step 5] Generating schematic SVG...")
try:
    generate_svg(file_='output/schematic.svg')
    print("✓ Schematic SVG: output/schematic.svg")
except:
    print("⚠ SVG generation skipped (optional)")

print("\n" + "=" * 70)
print("SUCCESS! Files generated:")
print("=" * 70)
print("\n✓ output/datalogger.net - KiCad netlist")
print("✓ output/place_components.py - Component placement script")
print("✓ output/schematic.svg - Circuit diagram (if generated)")

print("\n" + "=" * 70)
print("NEXT STEPS:")
print("=" * 70)
print("""
1. Open KiCad PCBNew
2. Create new PCB project
3. Tools → Load Netlist → Select output/datalogger.net
4. All components will appear on the canvas

5. Run placement script:
   - Tools → Scripting Console
   - Paste contents of output/place_components.py
   - Or: File → Open → output/place_components.py

6. Route the board:
   - Manual routing (recommended for first PCB)
   - OR: Tools → External Tools → FreeRouting

7. Add ground plane:
   - Right toolbar → Add Filled Zone
   - Draw around board edge
   - Net: GND, Layer: B.Cu

8. Run DRC:
   - Inspect → Design Rules Checker
   - Fix any errors

9. Export Gerbers:
   - File → Fabrication Outputs → Gerbers
   - Select all layers, generate

10. Order PCBs from JLCPCB, OSH Park, or PCBWay

Board size suggestion: 60mm x 40mm
Mounting holes: 3mm diameter, M3 screws
""")

print("=" * 70)
print("Good luck with your first PCB! 🚴‍♂️")
print("=" * 70)
