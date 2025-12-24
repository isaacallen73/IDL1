# PCB Design Checklist - Mountain Bike Datalogger

Use this checklist to ensure you haven't missed anything critical!

## Pre-Design Phase
- [ ] Read through entire KICAD_WORKFLOW.md guide
- [ ] Determine final board dimensions based on battery size
- [ ] Select specific waterproof connector models (J3, J4, J5)
- [ ] Confirm all component footprints are available
- [ ] Download component datasheets for reference

---

## Schematic Phase

### Power System
- [ ] Battery connector (J2) to XIAO BAT+ pad
- [ ] XIAO 3V3 output connected to all devices
- [ ] All GND symbols connected (common ground)
- [ ] Decoupling capacitor (10µF) at XIAO 3V3 output (C1)
- [ ] Decoupling capacitor at BMI160 VCC (C2)
- [ ] Decoupling capacitor at GPS VCC (C3)
- [ ] Decoupling capacitor at SD card VCC (C4)
- [ ] Decoupling capacitors at connectors (C5, C6, C7)
- [ ] Power LED circuit (D1 + R2) connected to 3V3

### SPI Bus (Shared Signals)
- [ ] XIAO D0 (GPIO0) → SCK on all SPI devices
- [ ] XIAO D1 (GPIO1) → MISO on all SPI devices
- [ ] XIAO D2 (GPIO2) → MOSI on all SPI devices
- [ ] Net labels added: `SPI_SCK`, `SPI_MISO`, `SPI_MOSI`

### SPI Chip Selects (Individual)
- [ ] XIAO D3 → BMI160 CS (on-board)
- [ ] XIAO D4 → Front connector Pin 6 (external IMU CS)
- [ ] XIAO D5 → Rear connector Pin 6 (external IMU CS)
- [ ] XIAO D6 → SD card CS

### GPS UART
- [ ] GPS TX → XIAO D7 (RX) - correct direction!
- [ ] GPS RX → XIAO D8 (TX) - correct direction!
- [ ] Net labels: `GPS_TX`, `GPS_RX`

### GPIO Signals
- [ ] XIAO D9 → Handlebar connector Pin 4 (LED_STATUS)
- [ ] XIAO D10 → R1 (10kΩ pull-up to 3V3) → Handlebar Pin 2 (BUTTON_PWR)
- [ ] XIAO A0 → Front connector Pin 7 (PRESSURE_FRONT)
- [ ] XIAO A2 → Rear connector Pin 7 (PRESSURE_REAR)

### Connectors
- [ ] Front connector (J3) - all 8 pins assigned correctly
- [ ] Rear connector (J4) - all 8 pins assigned correctly
- [ ] Handlebar connector (J5) - all 4 pins assigned correctly
- [ ] Battery connector (J2) - polarity marked clearly

### Electrical Rules
- [ ] Run ERC (Electrical Rules Check)
- [ ] All errors resolved (warnings OK if intentional)
- [ ] No floating power pins
- [ ] No unintended NC (no-connect) pins

---

## Footprint Assignment Phase

### Verify Each Component
- [ ] U1 (XIAO) - Custom footprint or temporary header
- [ ] U2 (BMI160) - Correct package (LGA-14 or breakout)
- [ ] U3 (GPS) - Module footprint matches purchased part
- [ ] J1 (microSD) - Footprint matches slot model
- [ ] J2 (JST) - JST-PH 2.0mm footprint
- [ ] J3, J4, J5 - Footprints match selected connectors
- [ ] C1-C7 - All 0805 package
- [ ] R1, R2, R3 - All 0805 package
- [ ] D1 (LED) - 0805 or THT as selected

### Double-Check Datasheets
- [ ] Pin numbering matches datasheet (especially ICs)
- [ ] Connector pinout matches (Pin 1 location)
- [ ] Mounting hole spacing correct (if using modules)

---

## PCB Layout Phase

### Board Setup
- [ ] Design rules configured (0.2mm clearance, 0.25mm trace)
- [ ] Net classes defined (Power: 0.5mm width)
- [ ] Board stackup: 2-layer, 1.6mm thickness
- [ ] Grid set to 1mm or 0.5mm

### Board Outline
- [ ] Rectangle drawn on Edge.Cuts layer
- [ ] Dimensions correct for battery + margin
- [ ] Mounting holes added (if needed)
- [ ] USB port accessibility confirmed

### Component Placement
- [ ] XIAO placed with USB accessible
- [ ] GPS module placed away from XIAO (antenna separation)
- [ ] SD card slot on accessible edge for insertion
- [ ] Battery connector near XIAO BAT pads
- [ ] Front/Rear connectors on edges (labeled clearly)
- [ ] On-board BMI160 placed away from GPS

### Decoupling Capacitor Placement (CRITICAL!)
- [ ] C1 within 3mm of XIAO 3V3 pin
- [ ] C2 within 3mm of BMI160 VCC pin
- [ ] C3 within 3mm of GPS VCC pin
- [ ] C4 within 3mm of SD card VCC pin
- [ ] C5, C6, C7 near respective connectors

### Routing
- [ ] All power connections routed (3V3, GND to each device)
- [ ] SPI bus routed (SCK, MISO, MOSI to all devices)
- [ ] SPI traces roughly equal length (within 10mm)
- [ ] All chip select lines routed
- [ ] UART lines routed (GPS TX/RX)
- [ ] Analog inputs routed (pressure sensors)
- [ ] GPIO routed (button, LED)
- [ ] No acute angles in traces (use 45° or rounded)

### Power Distribution
- [ ] Ground pour added on bottom layer (B.Cu)
- [ ] Ground vias placed every 5-10mm around perimeter
- [ ] Power traces 0.5mm width minimum
- [ ] Signal traces 0.25mm width minimum
- [ ] All GND pads connected to ground pour

### GPS Antenna Area
- [ ] GPS antenna area clear of copper pour on top layer
- [ ] Minimal ground under GPS antenna on bottom layer
- [ ] GPS module not blocked by tall components

### Design Rule Check (DRC)
- [ ] Run DRC - ZERO errors
- [ ] All nets connected (no "unconnected" errors)
- [ ] No clearance violations
- [ ] No track width violations
- [ ] Warnings reviewed and intentional

---

## Silkscreen & Documentation

### Labels
- [ ] "FRONT" near J3
- [ ] "REAR" near J4
- [ ] "HANDLEBAR" near J5
- [ ] "BATTERY +" polarity on J2
- [ ] "SD CARD" with insertion direction
- [ ] "GPS" labeled
- [ ] Board name and version (e.g., "Bike Logger v1.0")
- [ ] Your name/initials (optional)

### Reference Designators
- [ ] All components have visible ref designators
- [ ] No overlapping text
- [ ] Text size readable (1mm minimum)
- [ ] Silkscreen not over pads (warnings OK)

### Polarity Marks
- [ ] LED polarity marked (anode/cathode)
- [ ] Battery connector polarity clear
- [ ] Electrolytic caps marked (if used)

---

## Pre-Manufacturing Checks

### 3D Viewer
- [ ] Open 3D viewer - inspect all angles
- [ ] Components fit on board
- [ ] No overlapping components
- [ ] Connectors accessible
- [ ] Mounting holes clear

### Final DRC
- [ ] Run DRC one more time - ZERO errors
- [ ] Check for "islands" (isolated copper)
- [ ] Verify ground plane is continuous

### Gerber Verification
- [ ] Generate Gerber files (all layers)
- [ ] Generate drill files
- [ ] Upload to online Gerber viewer
- [ ] Verify board outline correct
- [ ] All pads present and correct size
- [ ] No missing traces
- [ ] Silkscreen readable

### Files to Generate
- [ ] Gerber files (.gbr) - all layers
- [ ] Drill files (.drl)
- [ ] BOM (Bill of Materials) CSV
- [ ] Assembly files (optional - for PCBA)
- [ ] Schematic PDF (for reference)

---

## Ordering Phase

### Manufacturer Selection
- [ ] Choose manufacturer (JLCPCB, OSH Park, PCBWay, etc.)
- [ ] Check lead times
- [ ] Check shipping costs

### Order Specifications
- [ ] Layers: 2
- [ ] Quantity: 5 (or more)
- [ ] Thickness: 1.6mm
- [ ] Copper weight: 1 oz
- [ ] Surface finish: HASL or ENIG
- [ ] Soldermask color: Green (or preference)
- [ ] Silkscreen color: White (standard)
- [ ] Remove order number: Yes (optional, small fee)

### Upload Files
- [ ] Zip all Gerber + drill files
- [ ] Upload to manufacturer
- [ ] Review auto-generated preview
- [ ] Confirm board dimensions match

### Final Review Before Payment
- [ ] Price reasonable (~$5-50 depending on mfg)
- [ ] Lead time acceptable
- [ ] Shipping method selected
- [ ] **PAUSE**: Have someone else review your design!
- [ ] Payment submitted

---

## Post-Order Checklist

### While Waiting for Boards
- [ ] Order components (from BOM_REFERENCE.csv)
- [ ] Order external harness components (BMI160s, pressure sensors)
- [ ] Prepare soldering workspace
- [ ] Review assembly order (smallest → largest components)
- [ ] Watch KiCad/soldering tutorials if needed

### When Boards Arrive
- [ ] Visual inspection for defects
- [ ] Verify board dimensions with caliper
- [ ] Check silkscreen legibility
- [ ] Verify hole sizes (test fit connectors)

---

## Assembly Phase

### Pre-Soldering
- [ ] Organize components by type
- [ ] Print out schematic for reference
- [ ] Set up soldering station (iron, flux, solder, tweezers)
- [ ] Test soldering iron on scrap PCB

### Soldering Order
- [ ] Solder resistors first (R1, R2, R3)
- [ ] Solder capacitors (C1-C7)
- [ ] Solder LED (D1) - watch polarity!
- [ ] Solder ICs (U2 BMI160 if bare chip)
- [ ] Solder connectors (J1-J5)
- [ ] Solder modules (U3 GPS)
- [ ] Solder XIAO last (U1) - use low-temp solder

### Post-Soldering Inspection
- [ ] Visual check for solder bridges
- [ ] Visual check for cold joints
- [ ] Multimeter continuity test (GND to GND)
- [ ] Multimeter resistance test (3V3 to GND = high impedance)

---

## Testing Phase

### Power-On Test (No Battery Yet!)
- [ ] Connect XIAO USB (without battery)
- [ ] Measure voltage at XIAO 3V3 pin (should be ~3.3V)
- [ ] Check for smoke/heat (STOP if detected!)
- [ ] Measure voltage at each device VCC pin

### Basic Firmware Test
- [ ] Program "Hello World" via USB
- [ ] Verify serial output
- [ ] Test GPIO toggle (blink LED)

### Device-by-Device Testing
- [ ] GPS UART test (read NMEA sentences)
- [ ] SD card SPI test (mount filesystem)
- [ ] BMI160 SPI test (read WHO_AM_I register)
- [ ] External connector continuity (test with multimeter)

### Full System Test
- [ ] Connect battery
- [ ] Run datalogger firmware
- [ ] Verify all sensors reporting data
- [ ] Test SD card logging
- [ ] Test external harnesses (when built)

---

## Common Issues & Fixes

### Issue: 3V3 rail shows 0V
- [ ] Check for short circuit (3V3 to GND)
- [ ] Check XIAO is properly soldered
- [ ] Check for solder bridge on XIAO pads

### Issue: Device not responding on SPI
- [ ] Verify chip select line routed correctly
- [ ] Check SPI bus wiring (SCK, MISO, MOSI)
- [ ] Verify device powered (measure VCC pin)
- [ ] Check device orientation (Pin 1 location)

### Issue: GPS not outputting data
- [ ] Verify TX/RX not swapped
- [ ] Check baud rate (9600 for NEO-6M)
- [ ] Wait for GPS fix (can take 1-2 minutes)
- [ ] Check antenna connection

### Issue: SD card not mounting
- [ ] Verify card is FAT32 formatted
- [ ] Check card detect pin (if used)
- [ ] Try different SD card
- [ ] Check SPI wiring

---

## Iteration Planning

### After First Prototype
- [ ] Document all issues found
- [ ] Note design improvements needed
- [ ] Update schematic with fixes
- [ ] Create v1.1 with corrections

### Future Enhancements
- [ ] Add custom XIAO footprint (castellated pads)
- [ ] Optimize board size/shape
- [ ] Add test points for debugging
- [ ] Consider 4-layer board for better routing
- [ ] Add TVS diodes for ESD protection
- [ ] Add reverse polarity protection on battery

---

## Success Criteria

Your PCB design is successful when:
- [ ] All devices power on correctly
- [ ] GPS outputs NMEA data
- [ ] SD card mounts and logs data
- [ ] On-board BMI160 reads accelerometer/gyro
- [ ] External connectors work with harnesses
- [ ] System runs continuously without errors
- [ ] Battery powers system for expected duration

---

**Congratulations!** You've completed your first PCB design! 🎉

Time to order your next revision with improvements, or scale up to production if it works perfectly!

---

*Last Updated: 2024*
*Project: Mountain Bike Datalogger v2*
*Designer: Isaac*
