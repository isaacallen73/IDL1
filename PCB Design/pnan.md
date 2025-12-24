Perfect! Here's a comprehensive summary of the netlist and design for your mountain bike datalogger:
Board Overview

    Main Controller: Seeed XIAO ESP32-C6 (castellated mount)
    On-board sensors: 1x BMI160 IMU, 1x NEO-6M GPS module
    Storage: microSD card slot
    Power: JST-PH 2.0mm battery connector, charges via XIAO USB-C
    External connections: 2x ~10-pin waterproof connectors (front/rear harnesses)

Power Distribution

Battery Input:

    Battery+ → XIAO BAT+ pad
    Battery- → GND (common ground plane)

3.3V Rail (from XIAO 3V3 pad):

    Powers: BMI160, NEO-6M, SD card, external sensors
    Add 10µF ceramic caps at each device

Decoupling capacitors:

    10µF at XIAO 3V3 output
    10µF at each BMI160
    10µF at GPS
    10µF at SD card
    All caps should be 0805 or 0603 size, placed close to device power pins

XIAO ESP32-C6 Pin Assignments
Pin	Function	Connected To
D0 (GPIO0)	SPI SCK	All BMI160s + SD Card SCK
D1 (GPIO1)	SPI MISO	All BMI160s + SD Card MISO
D2 (GPIO2)	SPI MOSI	All BMI160s + SD Card MOSI
D3 (GPIO3)	SPI CS	On-board BMI160 CS
D4 (GPIO4)	SPI CS	Front BMI160 CS (via front connector)
D5 (GPIO5)	SPI CS	Rear BMI160 CS (via rear connector)
D6 (GPIO6)	SPI CS	SD Card CS
D7 (GPIO7)	UART RX	GPS TX
D8 (GPIO20)	UART TX	GPS RX
D9 (GPIO21)	GPIO	Status LED (on front connector)
D10 (GPIO8)	GPIO	Button 1 (power on - via handlebar connector)
D11? (GPIO9?)	GPIO	Button 2 (power off - via handlebar connector)
A0 (GPIO22)	Analog/PWM	Front Pressure Sensor 1
A2 (GPIO2)	Analog/PWM	Rear Pressure Sensor 1
3V3	Power Out	Distribute to all devices
GND	Ground	Common ground plane
BAT+	Battery+	JST connector
BAT-	Battery-	GND

Note: GPIO2, GPIO8, GPIO9 are strapping pins - I've assigned GPIO8 to button (should be pulled high with 10kΩ), and kept GPIO2/3 for analog inputs which should be safe.
On-Board Components
1. BMI160 IMU (on-board)

    VCC → 3.3V (+ 10µF cap)
    GND → GND
    SCK → D0
    MISO → D1 (SDO)
    MOSI → D2 (SDI)
    CS → D3
    INT1, INT2 → not connected (or tie to GND)

2. NEO-6M GPS Module

    VCC → 3.3V (+ 10µF cap)
    GND → GND
    TX → D7 (XIAO RX)
    RX → D8 (XIAO TX)
    PPS → not connected (optional: could go to GPIO for timing)
    External antenna connector: Mount U.FL or SMA connector, route to GPS antenna pad

3. microSD Card Slot

    VCC → 3.3V (+ 10µF cap)
    GND → GND
    SCK → D0
    MISO → D1
    MOSI → D2
    CS → D6
    CD (card detect) → optional, or tie to GND

4. Status LED (on-board)

    Consider adding a power LED:
        Anode → 3.3V
        Cathode → 1kΩ resistor → GND
        Green or blue LED

5. Battery Connector

    JST-PH 2.0mm, 2-pin
    Pin 1 (red) → BAT+
    Pin 2 (black) → GND

Front Connector (8-pin waterproof)

Suggested: TE AMP MCP series or equivalent
Pin	Signal	Description
1	3.3V	Power supply
2	GND	Ground
3	SPI_SCK	D0 - shared clock
4	SPI_MISO	D1 - shared MISO
5	SPI_MOSI	D2 - shared MOSI
6	BMI_CS_FRONT	D4 - front IMU chip select
7	PRESSURE_FRONT	A0 - front pressure sensor
8	HALL_FRONT	A1 - front hall effect sensor


Pull-up resistor: 10kΩ from BUTTON_PWR (D10) to 3.3V on main board
Rear Connector (8-pin waterproof)
Pin	Signal	Description
1	3.3V	Power supply
2	GND	Ground
3	SPI_SCK	D0 - shared clock
4	SPI_MISO	D1 - shared MISO
5	SPI_MOSI	D2 - shared MOSI
6	BMI_CS_REAR	D5 - rear IMU chip select
7	PRESSURE_REAR	A2 - rear pressure sensor
8	HALL_REAR	A3 - rear hall effect sensor


Handlebar Connector
1   GND Ground
2	BUTTON_PWR	D10 - power button (pull to GND when pressed)
3	BUTTON_PWR	D11? - power button (pull to GND when pressed)
4	LED_STATUS	D9 - drives LED (active high)

PCB Layout Guidelines
Board Dimensions:

    Match your battery size (tell me dimensions and I'll suggest exact size)
    Add ~2mm margin around perimeter
    Standard 1.6mm thickness

Layer Stack:

    Top: Components + signals
    Bottom: Ground pour + some signals

Component Placement:

    XIAO C6: Near center or one edge for USB access
    GPS module: Top side, opposite corner from XIAO (antenna separation)
    On-board BMI160: Near XIAO, away from GPS
    SD card: Accessible edge for card insertion
    Connectors: Opposite edges (front/rear) or same edge with clear labeling
    Battery connector: Near XIAO BAT pads

Critical Routing:

    GPS antenna: Keep GPS antenna area clear of copper pours on top layer, minimal ground pour on bottom under antenna
    SPI traces: Keep SCK, MOSI, MISO roughly equal length, run parallel with ground nearby
    Ground plane: Solid pour on bottom layer, top layer fill where possible
    Decoupling caps: Place within 2-3mm of IC power pins
    USB access: Ensure XIAO USB-C port is accessible for programming/charging

Design Rules:

    Trace width: 0.25mm for signals, 0.5mm for power
    Clearance: 0.2mm minimum
    Via size: 0.6mm drill, 1.0mm pad

Bill of Materials (BOM)
Qty	Part	Part Number / Description
1	XIAO ESP32-C6	Seeed 113991054
3	BMI160 IMU	Bosch BMI160 breakout or bare IC
1	NEO-6M GPS	u-blox NEO-6M module
1	microSD card slot	Any standard push-push slot
1	JST-PH 2.0mm connector	2-pin, right angle
2	Waterproof connectors	TE AMP MCP 10-pin (or Molex MicroFit)
~8	10µF ceramic cap	0805 size, X7R or X5R
1	1kΩ resistor	0805, for power LED
1	10kΩ resistor	0805, button pull-up
1	LED (green)	0805 or 3mm through-hole
1	U.FL or SMA connector	For GPS external antenna
External Harness Details
Front Harness (connects to front connector):

    1x BMI160 on small PCB
    1x Pressure transducers (3.3V analog output)
    1x Hall effect sensor provisions (for future)
    Waterproof cable with mating connector

Rear Harness (connects to rear connector):

    1x BMI160 on small PCB
    1x Pressure transducer
    1x Hall effect sensor provisions (for future)
    Waterproof cable with mating connector

Handlebar Harness
    2x Power button (momentary, NO)
    1x Status LED (with current-limiting resistor ~220Ω to GND)

Notes for KiCad Implementation

    XIAO footprint: Create custom footprint with castellated pads (1mm x 1.5mm pads, 2.54mm pitch)
    GPS keepout: Add keepout zone under GPS antenna on top copper
    Ground stitching: Add vias around perimeter to connect top/bottom ground pours
    Silkscreen: Label all connectors clearly, add polarity marks
    Mounting: Consider Hammond 1551 series enclosures (various sizes available)
