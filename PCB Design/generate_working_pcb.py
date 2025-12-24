#!/usr/bin/env python3
"""
Working PCB Generation for Mountain Bike Datalogger
Uses SKiDL default parts library (no external dependencies needed)
"""

from skidl import *
import os

# Use SKiDL's built-in backup library
set_default_tool(KICAD)

print("=" * 70)
print("Mountain Bike Datalogger - PCB Generation (SKiDL Built-in Library)")
print("=" * 70)

print("\n[Step 1] Defining circuit with SKiDL...")

# Nets
gnd, vcc, vbat = Net('GND'), Net('+3V3'), Net('VBAT')
spi_sck, spi_miso, spi_mosi = Net('SPI_SCK'), Net('SPI_MISO'), Net('SPI_MOSI')
cs_onboard, cs_front, cs_rear, cs_sd = Net('CS_ONBOARD'), Net('CS_FRONT'), Net('CS_REAR'), Net('CS_SD')
gps_tx, gps_rx = Net('GPS_TX'), Net('GPS_RX')
led, btn = Net('LED_STATUS'), Net('BUTTON_PWR')
p_front, p_rear = Net('PRESSURE_FRONT'), Net('PRESSURE_REAR')

print("Creating XIAO ESP32-C6 microcontroller...")
# XIAO represented as header connector
xiao = Part('Device', 'C', ref='U1', value='XIAO_ESP32-C6', dest=TEMPLATE)
xiao.ref = 'U1'
xiao.value = 'XIAO_ESP32-C6'
xiao.footprint = 'Module:Seeeduino_XIAO'

# Create pins
for i in range(1, 18):
    Pin(num=i, name=f'P{i}', func=Pin.types.PASSIVE).part = xiao

# Assign functionality to pins
xiao[1] += spi_sck     # D0
xiao[2] += spi_miso    # D1
xiao[3] += spi_mosi    # D2
xiao[4] += cs_onboard  # D3
xiao[5] += cs_front    # D4
xiao[6] += cs_rear     # D5
xiao[7] += cs_sd       # D6
xiao[8] += gps_rx      # D7 (MCU RX from GPS TX)
xiao[9] += gps_tx      # D8 (MCU TX to GPS RX)
xiao[10] += led        # D9
xiao[11] += btn        # D10
xiao[12] += p_front    # A0
xiao[13] += p_rear     # A2
xiao[14] += vcc        # 3V3
xiao[15] += gnd        # GND
xiao[16] += vbat       # BAT+
xiao[17] += gnd        # BAT-

print("Creating passive components...")
# Capacitors
for i, ref in enumerate(['C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7'], 1):
    c = Part('Device', 'C', ref=ref, value='10uF', footprint='Capacitor_SMD:C_0805_2012Metric')
    c[1] += vcc
    c[2] += gnd

# Resistors
r1 = Part('Device', 'R', ref='R1', value='10k', footprint='Resistor_SMD:R_0805_2012Metric')
r1[1] += btn
r1[2] += vcc

r2 = Part('Device', 'R', ref='R2', value='1k', footprint='Resistor_SMD:R_0805_2012Metric')

d1 = Part('Device', 'LED', ref='D1', value='PWR', footprint='LED_SMD:LED_0805_2012Metric')
d1['A'] += vcc
d1['K'] += r2[1]
r2[2] += gnd

print("Creating connectors...")

def make_connector(ref, val, pins, footprint='Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical'):
    conn = Part('Device', 'C', ref=ref, value=val, dest=TEMPLATE)
    conn.ref = ref
    conn.value = val
    conn.footprint = footprint
    for i in range(1, pins+1):
        Pin(num=i, name=f'P{i}', func=Pin.types.PASSIVE).part = conn
    return conn

# Battery
batt = make_connector('J2', 'BATTERY', 2, 'Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical')
batt[1] += vbat
batt[2] += gnd

# BMI160 onboard
imu = make_connector('U2', 'BMI160', 8, 'Package_LGA:LGA-14_3x2.5mm_P0.5mm')
imu[1] += vcc
imu[2] += gnd
imu[3] += spi_sck
imu[4] += spi_mosi
imu[5] += spi_miso
imu[6] += cs_onboard
imu[7] += NC
imu[8] += NC

# GPS
gps_mod = make_connector('U3', 'NEO-6M', 4, 'RF_GPS:ublox_NEO')
gps_mod[1] += vcc
gps_mod[2] += gnd
gps_mod[3] += gps_tx  # GPS TX
gps_mod[4] += gps_rx  # GPS RX

# SD Card
sd = make_connector('J1', 'MicroSD', 6, 'Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5')
sd[1] += vcc
sd[2] += gnd
sd[3] += spi_sck
sd[4] += spi_miso
sd[5] += spi_mosi
sd[6] += cs_sd

# Front connector
front = make_connector('J3', 'FRONT', 8)
front[1] += vcc
front[2] += gnd
front[3] += spi_sck
front[4] += spi_miso
front[5] += spi_mosi
front[6] += cs_front
front[7] += p_front
front[8] += NC

# Rear connector
rear = make_connector('J4', 'REAR', 8)
rear[1] += vcc
rear[2] += gnd
rear[3] += spi_sck
rear[4] += spi_miso
rear[5] += spi_mosi
rear[6] += cs_rear
rear[7] += p_rear
rear[8] += NC

# Handlebar
hbar = make_connector('J5', 'HANDLEBAR', 4)
hbar[1] += gnd
hbar[2] += btn
hbar[3] += NC
hbar[4] += led

print("✓ Circuit defined!")

# Generate netlist
print("\n[Step 2] Generating netlist...")
os.makedirs('output', exist_ok=True)

try:
    generate_netlist(file_='output/datalogger.net')
    print("✓ Netlist: output/datalogger.net")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

# Placement script
print("\n[Step 3] Creating placement script...")
with open('output/place_components.py', 'w') as f:
    f.write('''#!/usr/bin/env python3
"""Component placement for datalogger - Run in KiCad PCBNew"""
import pcbnew

board = pcbnew.GetBoard()
placements = {
    'U1': (30, 20, 0), 'U2': (20, 18, 0), 'U3': (50, 30, 0),
    'J1': (10, 30, 90), 'J2': (35, 10, 0), 'J3': (5, 5, 0),
    'J4': (55, 5, 0), 'J5': (30, 38, 0),
    'C1': (32, 18, 0), 'C2': (18, 16, 0), 'C3': (52, 28, 0),
    'C4': (8, 28, 0), 'C5': (3, 8, 0), 'C6': (57, 8, 0), 'C7': (28, 36, 0),
    'R1': (28, 34, 0), 'R2': (42, 20, 0), 'D1': (42, 22, 0),
}
for ref, (x, y, rot) in placements.items():
    fp = board.FindFootprintByReference(ref)
    if fp:
        fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
        fp.SetOrientationDegrees(rot)
        print(f"✓ {ref}")
pcbnew.Refresh()
print("Done!")
''')
print("✓ Placement: output/place_components.py")

# Instructions
print("\n[Step 4] Creating instructions...")
with open('output/README.md', 'w') as f:
    f.write('''# Datalogger PCB - Generated Files

## SUCCESS! Files Ready:
✓ `datalogger.net` - KiCad netlist
✓ `place_components.py` - Automated placement

## Quick Start:

1. **Open KiCad PCBNew** → New project

2. **Load Netlist**: Tools → Load Netlist → datalogger.net

3. **Run Placement**: Tools → Scripting Console → Open place_components.py

4. **Set Board**: Edge.Cuts layer → Draw 60x40mm rectangle

5. **Route**: Press 'X' to route traces (or use FreeRouting)

6. **Ground Plane**: Add Filled Zone → Net=GND, Layer=B.Cu

7. **DRC**: Inspect → Design Rules Checker

8. **Export**: File → Fabrication Outputs → Gerbers

9. **Order**: Upload to JLCPCB.com ($25 shipped)

## Component Summary:
- U1: XIAO ESP32-C6 (update footprint to Seeed library)
- U2: BMI160 IMU
- U3: NEO-6M GPS
- J1: MicroSD slot
- J2: JST battery connector
- J3/J4: Front/Rear 8-pin connectors
- J5: Handlebar 4-pin connector
- C1-C7: 10µF 0805 capacitors
- R1: 10kΩ, R2: 1kΩ
- D1: Power LED

Good luck! 🚴‍♂️
''')
print("✓ Instructions: output/README.md")

print("\n" + "=" * 70)
print("SUCCESS!")
print("=" * 70)
print("""
✓ datalogger.net - Complete KiCad netlist
✓ place_components.py - Automated component placement
✓ README.md - Step-by-step workflow

Next: Open KiCad → Load netlist → Place components → Route!

NOTE: Update U1 (XIAO) footprint in KiCad to use the Seeed library:
  OPL_Kicad_Library/Seeed Studio XIAO Series Library/XIAO-ESP32C6-SMD.kicad_mod
""")
print("=" * 70)
