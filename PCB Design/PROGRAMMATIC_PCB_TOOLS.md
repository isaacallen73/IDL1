# Programmatic PCB Design Tools - Research Summary

## Overview

This document summarizes command-line and programmatic tools for PCB design that can be used to create PCBs from specifications. The focus is on practical, working solutions for generating the mountain bike datalogger PCB.

---

## 1. KiCad CLI (`kicad-cli`)

### What It Is
KiCad's official command-line interface for automation tasks, available in KiCad 7.0+, with significant enhancements in 8.0 and 9.0.

### Key Capabilities

**PCB Operations:**
- Design Rule Check (DRC) with report generation
- Export to multiple formats:
  - Gerber files (one per layer)
  - PDF files
  - DXF files
  - 3D files (STEP, GLB/GLTF)
  - Position files (for assembly)
- **New in KiCad 9.0**: Raytracing 3D images

**Schematic Operations:**
- PDF export
- Netlist generation
- BOM generation

**Limitations:**
- **Cannot create or modify designs** - only exports from existing files
- No automatic placement or routing
- No schematic creation from code

### Use Cases
- CI/CD pipelines for PCB projects
- Automated Gerber generation
- Batch processing of designs
- Documentation generation

### Example Commands
```bash
# Export Gerbers
kicad-cli pcb export gerbers --output ./gerbers design.kicad_pcb

# Export drill files
kicad-cli pcb export drill --output ./gerbers design.kicad_pcb

# Run DRC
kicad-cli pcb drc --output drc_report.txt design.kicad_pcb

# Export 3D STEP file
kicad-cli pcb export step --output design.step design.kicad_pcb

# Export PDF schematic
kicad-cli sch export pdf --output schematic.pdf design.kicad_sch
```

### Documentation
- [KiCad CLI Documentation (v9.0)](https://docs.kicad.org/9.0/en/cli/cli.html)
- [KiCad CLI Documentation (v8.0)](https://docs.kicad.org/8.0/en/cli/cli.html)

**Verdict:** Excellent for the **last step** (Gerber export), but cannot generate PCBs from scratch.

---

## 2. SKiDL (Python-Based Schematic Entry)

### What It Is
A Python package that allows you to describe electronic circuits using code instead of graphical schematic editors. Think "infrastructure as code" for electronics.

### Key Features
- **Schematic capture in Python code**
- Electrical rules checking (ERC)
- Multiple output formats:
  - KiCad netlist (`.net`)
  - Direct PCB file (`.kicad_pcb`) generation
  - SVG schematic generation (KiCad 6, 7, 8)
  - XML BOM
- SPICE integration for simulation
- Version control friendly (text-based)
- Code reviews and diffs for circuits

### Workflow
1. Write circuit in Python using SKiDL
2. Generate netlist or direct PCB file
3. Import to KiCad PCBNew (if using netlist)
4. Layout components and route
5. Export Gerbers

### Example Code
```python
from skidl import *

# Create circuit
gnd = Net('GND')
vcc = Net('VCC')

# Add microcontroller
mcu = Part('MCU_Module', 'Arduino_Nano', footprint='Module:Arduino_Nano')
mcu['GND'] += gnd
mcu['VCC'] += vcc

# Add LED with resistor
led = Part('Device', 'LED', footprint='LED_SMD:LED_0805_2012Metric')
res = Part('Device', 'R', value='220', footprint='Resistor_SMD:R_0805_2012Metric')

led['A'] += mcu['D13']
led['K'] += res[1]
res[2] += gnd

# Generate netlist
generate_netlist()
```

### Latest Version (2025)
- Supports KiCad 5, 6, 7, and 8
- Active development
- Available on PyPI: `pip install skidl`

### Documentation
- [Official Documentation](https://devbisme.github.io/skidl/)
- [GitHub Repository](https://github.com/devbisme/skidl)
- [PyPI Package](https://pypi.org/project/skidl/)

**Verdict:** **Excellent for step 1** (schematic/netlist generation). Replaces manual schematic entry. Still requires manual or automated routing.

---

## 3. KiCad Python API (pcbnew)

### What It Is
KiCad's built-in Python scripting interface for the PCB editor (pcbnew). Allows programmatic manipulation of PCB layouts.

### Two APIs Available

#### A. Legacy SWIG-based Python Bindings
- Available in KiCad 5-9
- **In maintenance mode** as of KiCad 9
- Direct access to `pcbnew` module
- Works within KiCad GUI or headless

#### B. New IPC API (KiCad 9+)
- **Public beta** with KiCad 9.0 (released ~February 2025)
- Modern API design
- Can be used from external Python scripts
- Python package: `kicad-python` by atait

### Capabilities
- **Create and modify PCB files**
- **Component placement** with coordinate control
- Add traces, vias, zones
- Create copper pours (ground planes)
- Manipulate footprints
- Read/write board properties
- Generate output files

### Use Cases
- Automated component placement
- Programmatic routing (simple traces)
- Custom DRC scripts
- Net length validation
- BOM generation
- Panelization

### Example Code (Legacy SWIG API)
```python
import pcbnew

# Load board
board = pcbnew.LoadBoard('design.kicad_pcb')

# Place component at specific location
footprint = board.FindFootprintByReference('U1')
footprint.SetPosition(pcbnew.wxPointMM(30, 20))
footprint.SetOrientation(90 * 10)  # 90 degrees (in decidegrees)

# Save board
board.Save('design.kicad_pcb')
```

### Example Code (kicad-python by atait)
```python
from kicad import KicadPCB

# Cross-version compatible
pcb = KicadPCB('design.kicad_pcb')
pcb.place_component('U1', x=30, y=20, rotation=90)
pcb.save()
```

### Important Limitations
- **Cannot do automatic routing** (must be done manually or with external tools)
- **Don't modify PCB while open in KiCad GUI** (file conflicts)
- Complex for beginners
- Limited documentation

### Third-Party Tools

#### kicad-python by atait
- Cross-version compatibility (KiCad 5-9)
- More Pythonic interface
- Headless operation support
- GUI integration examples
- Install: `pip install kicad-python`

### Documentation
- [KiCad Python Bindings Docs](https://dev-docs.kicad.org/en/apis-and-binding/pcbnew/index.html)
- [kicad-python GitHub](https://github.com/atait/kicad-python)
- [Community Tutorial](https://www.prasunbarua.com/2025/04/automating-pcb-design-workflows-with.html)

**Verdict:** **Excellent for step 2-3** (component placement, basic PCB manipulation). Still requires routing.

---

## 4. PCBflow

### What It Is
A Python-based PCB layout package focused on **programmatic routing**. Based on CuFlow by James Bowman.

### Key Features
- Define board geometry in code
- **River routing** - Turtle graphics-like commands for traces
- Copper pour management
- Best used as companion to SKiDL
- Outputs KiCad PCB files

### Status
- **Alpha quality** - not fully documented
- Active development
- Open source

### Workflow
1. Define circuit with SKiDL
2. Create Board with PCBflow
3. Route traces using "river" commands
4. Fill copper pours
5. Export to KiCad format

### Example Code
```python
from pcbflow import *

# Create 40mm x 30mm board
brd = Board((40, 30))
brd.add_outline()

# Fill copper layers with GND
brd.fill_layer("GTL", "GND")  # Top layer
brd.fill_layer("GBL", "GND")  # Bottom layer

# Route traces (Turtle graphics style)
brd.DC((x1, y1))  # Start position
brd.forward(10)
brd.left(90)
brd.forward(5)
```

### Real-World Example
Complete macropad project done entirely in Python:
- Schematic capture with SKiDL
- Routing with PCBflow
- Case design in Python
- See: [Hackaday Article](https://hackaday.com/2023/01/02/the-whole-thing-in-python/)

### Installation
```bash
git clone https://github.com/michaelgale/pcbflow.git
cd pcbflow
python setup.py install
```

### Documentation
- [GitHub Repository](https://github.com/michaelgale/pcbflow)
- [README](https://github.com/michaelgale/pcbflow/blob/main/README.md)

**Verdict:** **Promising for step 4** (routing), but alpha quality. Best for simple, programmatic routing patterns. May not handle complex boards well.

---

## 5. FreeRouting (Autorouter)

### What It Is
Open-source Java-based autorouter for PCBs. Can be used as KiCad plugin or standalone.

### Key Features (2025 Updates)
- **New Python client library** (v2.1.0+)
- **Public API** for programmatic access
- **CLI support** with JSON output
- Integrates with KiCad, EasyEDA, tscircuit
- Respects design rules from KiCad

### KiCad Integration
```
Tools → External Plugins → FreeRouting
```

### CLI Usage
```bash
# Run autorouting from command line
freerouting input.dsn output.ses

# Get results in JSON format
freerouting --json input.dsn
```

### Python API (New in 2025)
```python
# Example conceptual usage (check latest docs for actual API)
from freerouting import FreeRouter

router = FreeRouter('design.dsn')
router.set_rules(clearance=0.2, trace_width=0.25)
result = router.route()
result.export('design.ses')
```

### Alternative: OrthoRoute (GPU-Accelerated)
- Built specifically for KiCad using IPC API
- GPU acceleration for faster routing
- Python-based
- Manhattan (orthogonal) routing
- See: [GitHub](https://github.com/bbenchoff/OrthoRoute)

### Documentation
- [FreeRouting Documentation](https://freerouting.org/freerouting/using-with-kicad)
- [GitHub Releases](https://github.com/freerouting/freerouting/releases)
- [OrthoRoute](https://bbenchoff.github.io/pages/OrthoRoute.html)

**Verdict:** **Good for step 4** (routing). FreeRouting with Python API enables full automation. OrthoRoute is modern alternative with GPU acceleration.

---

## 6. PHDL (Printed Circuit Hardware Description Language)

### What It Is
An HDL (Hardware Description Language) for PCB design, similar to VHDL/Verilog but for circuit boards instead of FPGAs.

### Features
- Text-based circuit description language
- Compiles to netlist format
- Supports PADS and EAGLE netlists
- Extensible output formats via Java
- Open source

### Status
- Version 2.1 (stable but older)
- Available as Eclipse plugin or CLI
- Less active development than other tools

### Workflow
1. Write circuit in PHDL syntax
2. Compile to netlist
3. Import netlist to layout tool (PADS, EAGLE, KiCad)
4. Layout and route

### Installation
Download from [SourceForge](https://sourceforge.net/projects/phdl/)

### Documentation
- [PHDL Website](https://phdl.sourceforge.net/2.0/about.php)
- [Technical Paper (OSTI)](https://www.osti.gov/servlets/purl/1078729)

**Verdict:** **Alternative to SKiDL** for step 1 (netlist generation). Less active community and fewer features than SKiDL. SKiDL is recommended instead.

---

## Complete Workflow Options

### Option 1: SKiDL + KiCad Python API + FreeRouting (RECOMMENDED)

**Pros:**
- Fully open source
- Strong community support
- Well-documented
- Works with latest KiCad

**Steps:**
1. **Circuit definition**: SKiDL (Python)
2. **Netlist generation**: SKiDL → `.net` file
3. **PCB creation & placement**: pcbnew Python API
4. **Routing**: FreeRouting with Python API
5. **Gerber export**: kicad-cli

**Automation Level:** ~80% - Still may need manual tweaking

**Code Example:**
```python
# Step 1: Define circuit
from skidl import *
# ... circuit definition ...
generate_netlist(file_='design.net')

# Step 2: Create PCB and place components
import pcbnew
board = pcbnew.BOARD()
# ... load netlist, place components ...
board.Save('design.kicad_pcb')

# Step 3: Autoroute
from freerouting import FreeRouter
router = FreeRouter('design.kicad_pcb')
router.route()

# Step 4: Export Gerbers
import subprocess
subprocess.run(['kicad-cli', 'pcb', 'export', 'gerbers', 'design.kicad_pcb'])
```

---

### Option 2: SKiDL + PCBflow (EXPERIMENTAL)

**Pros:**
- Fully programmatic routing
- Everything in Python
- Version control friendly

**Cons:**
- PCBflow is alpha quality
- Limited to simple routing patterns
- Less mature than KiCad tools

**Steps:**
1. **Circuit definition**: SKiDL
2. **Board creation & routing**: PCBflow
3. **Export**: PCBflow → KiCad → kicad-cli

**Automation Level:** ~90% - More automated but less reliable

---

### Option 3: Manual Layout with Python Assistance

**Most Practical for Complex Boards:**

1. **Generate netlist**: SKiDL (automated)
2. **Import to KiCad**: Manual
3. **Component placement**: Python script with pcbnew (semi-automated)
4. **Routing**: Manual in KiCad (highest quality)
5. **DRC and Gerbers**: kicad-cli (automated)

**Automation Level:** ~40% - But highest quality results

---

## Complete Example Script

See `generate_pcb.py` in this directory for a complete working example that:
1. Defines the datalogger circuit in SKiDL
2. Generates netlist
3. Creates component placement script
4. Provides kicad-cli commands for Gerber export

**Usage:**
```bash
pip install skidl kicad-python
python generate_pcb.py
```

---

## Tool Comparison Matrix

| Tool | Purpose | Maturity | Automation | Learning Curve | Recommended |
|------|---------|----------|------------|----------------|-------------|
| **kicad-cli** | Export/DRC | Stable | High | Low | ✓ Yes |
| **SKiDL** | Schematic/Netlist | Stable | High | Medium | ✓ Yes |
| **pcbnew API** | PCB Manipulation | Stable | Medium | High | ✓ Yes |
| **kicad-python** | Wrapper for pcbnew | Active | Medium | Medium | ✓ Yes |
| **FreeRouting** | Autorouting | Stable | Medium | Medium | ✓ Yes |
| **OrthoRoute** | GPU Autorouting | New | Medium | Medium | Maybe |
| **PCBflow** | Programmatic Routing | Alpha | High | High | Maybe |
| **PHDL** | Netlist Generation | Older | High | High | No (use SKiDL) |

---

## Recommended Toolchain for Your Datalogger

### For Full Automation (80% automated):
```
SKiDL → pcbnew Python API → FreeRouting → kicad-cli
```

### For Highest Quality (40% automated):
```
SKiDL → KiCad GUI → Manual routing → kicad-cli
```

### For Learning/Experimentation:
```
SKiDL → PCBflow → kicad-cli
```

---

## Installation Instructions

### Install KiCad 8 or 9
```bash
# Download from official website
# https://www.kicad.org/download/

# Or use package manager (Linux)
sudo apt install kicad
```

### Install Python Tools
```bash
# Create virtual environment
python -m venv pcb-env
source pcb-env/bin/activate  # On Windows: pcb-env\Scripts\activate

# Install core tools
pip install skidl
pip install kicad-python

# Optional: PCBflow (experimental)
git clone https://github.com/michaelgale/pcbflow.git
cd pcbflow
python setup.py install
```

### Install FreeRouting
```bash
# Download latest release
# https://github.com/freerouting/freerouting/releases

# Or install as KiCad plugin via Plugin Manager
```

---

## Next Steps for Your Project

1. **Start with the provided `generate_pcb.py` script**
   - Review and customize the circuit definition
   - Add missing components (hall sensors, etc.)
   - Adjust footprints to match your actual parts

2. **Generate initial netlist**
   ```bash
   python generate_pcb.py
   ```

3. **Import to KiCad and review**
   - Open KiCad PCBNew
   - Tools → Load Netlist
   - Select `output/datalogger.net`

4. **Run placement script** (or do manually)
   - Use the generated `output/place_components.py`
   - Adjust positions as needed

5. **Route the board**
   - Try FreeRouting first
   - Manual touch-up as needed
   - Pay attention to GPS antenna clearance

6. **Export Gerbers**
   ```bash
   kicad-cli pcb export gerbers --output ./gerbers output/datalogger.kicad_pcb
   ```

---

## Sources and References

### Official Documentation
- [KiCad Command-Line Interface](https://docs.kicad.org/9.0/en/cli/cli.html)
- [KiCad Python Bindings](https://dev-docs.kicad.org/en/apis-and-binding/pcbnew/index.html)
- [SKiDL Documentation](https://devbisme.github.io/skidl/)
- [FreeRouting Documentation](https://freerouting.org/freerouting/using-with-kicad)

### GitHub Repositories
- [SKiDL](https://github.com/devbisme/skidl)
- [kicad-python](https://github.com/atait/kicad-python)
- [PCBflow](https://github.com/michaelgale/pcbflow)
- [FreeRouting](https://github.com/freerouting/freerouting)
- [OrthoRoute](https://github.com/bbenchoff/OrthoRoute)
- [PHDL](https://sourceforge.net/projects/phdl/)

### Tutorials and Articles
- [Automating PCB Design Workflows with Python](https://www.prasunbarua.com/2025/04/automating-pcb-design-workflows-with.html)
- [Programmatic Layout with KiCad and Python](https://jeffmcbride.net/programmatic-layout-with-kicad-and-python/)
- [Hackaday: The Whole Thing In Python](https://hackaday.com/2023/01/02/the-whole-thing-in-python/)
- [Hackaday: SKiDL Article](https://hackaday.com/2016/12/28/skidl-script-your-circuits-in-python/)
- [Autorouting in KiCad using FreeRouting](https://www.protoexpress.com/blog/how-to-autoroute-pcb-layout-in-kicad-using-freerouting-plugin/)

---

**Last Updated:** 2025-12-21
**KiCad Version:** 9.0
**Python Version:** 3.8+
