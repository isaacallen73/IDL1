# PCB Design for Mountain Bike Datalogger

This directory contains everything needed to design and manufacture the PCB for the mountain bike datalogger project.

---

## Quick Links

| Document | Purpose |
|----------|---------|
| **[QUICK_START.md](QUICK_START.md)** | Start here! Step-by-step guide to generate your PCB |
| **[PROGRAMMATIC_PCB_TOOLS.md](PROGRAMMATIC_PCB_TOOLS.md)** | Complete research on CLI/Python PCB tools |
| **[pnan.md](pnan.md)** | Complete circuit specification and netlist |
| **[KICAD_WORKFLOW.md](KICAD_WORKFLOW.md)** | Manual KiCad design workflow |
| **[DESIGN_CHECKLIST.md](DESIGN_CHECKLIST.md)** | Pre-manufacturing verification checklist |

---

## What's in This Directory

### Documentation Files

1. **QUICK_START.md** - Your starting point
   - Three workflow options (automated, semi-automated, manual)
   - Step-by-step walkthrough
   - Troubleshooting guide
   - Complete from netlist generation to ordering PCBs

2. **PROGRAMMATIC_PCB_TOOLS.md** - Tool research and comparison
   - kicad-cli capabilities and examples
   - SKiDL for Python-based schematic entry
   - pcbnew Python API for PCB manipulation
   - FreeRouting autorouter integration
   - PCBflow for programmatic routing
   - PHDL hardware description language
   - Complete workflow options with pros/cons
   - Installation instructions

3. **pnan.md** - Circuit specification
   - Complete netlist and pin assignments
   - Power distribution
   - Component list
   - Connector pinouts
   - PCB layout guidelines
   - Bill of materials

4. **KICAD_WORKFLOW.md** - Manual KiCad workflow
   - Traditional PCB design process
   - Best practices for KiCad
   - Layer stack configuration
   - Design rules

5. **DESIGN_CHECKLIST.md** - Pre-manufacturing checklist
   - Component placement verification
   - Routing checks
   - DRC validation
   - Gerber export checklist

### Python Scripts

1. **generate_pcb.py** - Circuit definition and netlist generator
   - Complete datalogger circuit in SKiDL
   - Generates KiCad-compatible netlist
   - Creates component placement helper script
   - Provides workflow examples
   - **Run this first!**

2. **autoroute_example.py** - Automation and export tools
   - FreeRouting integration
   - DRC automation
   - Gerber export automation
   - Position file generation
   - 3D model export

---

## Getting Started

### Option 1: Quick Start (Recommended)

```bash
# Install dependencies
pip install skidl

# Generate circuit and netlist
python generate_pcb.py

# Follow the steps in QUICK_START.md
```

### Option 2: Read First, Then Build

1. Read **PROGRAMMATIC_PCB_TOOLS.md** to understand the tools
2. Review **pnan.md** to understand the circuit
3. Follow **QUICK_START.md** for step-by-step instructions

### Option 3: Traditional Workflow

1. Read **KICAD_WORKFLOW.md**
2. Use **pnan.md** as reference
3. Design manually in KiCad GUI
4. Use **DESIGN_CHECKLIST.md** before ordering

---

## Workflow Comparison

### Fully Automated (Experimental)
- **Tools**: SKiDL + pcbnew API + FreeRouting + kicad-cli
- **Automation**: ~80%
- **Quality**: Good
- **Time**: 1-2 hours
- **Best for**: Simple boards, rapid prototyping, learning

### Semi-Automated (Recommended)
- **Tools**: SKiDL + Manual KiCad + FreeRouting + kicad-cli
- **Automation**: ~40%
- **Quality**: Excellent
- **Time**: 2-4 hours
- **Best for**: Production boards, this datalogger

### Manual with Scripts
- **Tools**: SKiDL (netlist only) + Manual KiCad + kicad-cli
- **Automation**: ~20%
- **Quality**: Excellent
- **Time**: 4-6 hours
- **Best for**: Learning KiCad, complex boards

---

## File Outputs

When you run `generate_pcb.py`, it creates:

```
output/
├── datalogger.net          # KiCad netlist (import to PCBNew)
├── place_components.py     # Component placement script
└── README.txt              # Next steps

```

When you run `autoroute_example.py export`, it creates:

```
output/
├── gerbers/
│   ├── *.gbr               # Gerber files (one per layer)
│   ├── *.drl               # Drill files
│   └── *-job.gbrjob        # Gerber job file
├── datalogger_pos.csv      # Component position file
├── datalogger.step         # 3D STEP model
└── drc_report.txt          # Design rule check report
```

---

## Key Technologies Used

### 1. SKiDL (Schematic Capture)
Python package for circuit definition
- Website: https://devbisme.github.io/skidl/
- GitHub: https://github.com/devbisme/skidl
- Install: `pip install skidl`

### 2. KiCad (PCB Design)
Open-source EDA software
- Website: https://www.kicad.org/
- Docs: https://docs.kicad.org/
- Version: 8.0 or 9.0 recommended

### 3. kicad-cli (Command Line Interface)
Built into KiCad 7+
- Export Gerbers
- Run DRC
- Generate documentation
- No separate installation needed

### 4. FreeRouting (Autorouter)
Open-source PCB autorouter
- Website: https://freerouting.org/
- GitHub: https://github.com/freerouting/freerouting
- Install: Via KiCad Plugin Manager

### 5. pcbnew Python API (Optional)
KiCad's Python scripting interface
- Built into KiCad
- Cross-version wrapper: `pip install kicad-python`
- Docs: https://dev-docs.kicad.org/

---

## Circuit Overview

**Main Components:**
- Seeed XIAO ESP32-C6 (microcontroller)
- 3x BMI160 IMU (onboard + front + rear)
- NEO-6M GPS module
- microSD card slot
- JST battery connector
- 3x waterproof connectors (front, rear, handlebar)

**Key Features:**
- SPI bus for sensors and SD card
- UART for GPS
- Analog inputs for pressure sensors
- GPIO for buttons and LED
- Complete power distribution with decoupling

**Board Size:** ~60mm x 40mm (adjust to battery)

**Layers:** 2 (top copper + bottom ground plane)

See [pnan.md](pnan.md) for complete specifications.

---

## Next Steps

1. **Generate netlist**
   ```bash
   python generate_pcb.py
   ```

2. **Import to KiCad**
   - Open KiCad PCBNew
   - Tools → Load Netlist
   - Select `output/datalogger.net`

3. **Place components**
   - Manual placement (recommended)
   - Or use `output/place_components.py`

4. **Route board**
   - Manual critical traces
   - FreeRouting for remainder

5. **Export Gerbers**
   ```bash
   python autoroute_example.py export datalogger.kicad_pcb
   ```

6. **Order PCBs**
   - Upload Gerbers to JLCPCB, PCBWay, or OSH Park
   - ~$5-20 for 5 boards
   - 1-2 week delivery

---

## Troubleshooting

**"SKiDL not found"**
```bash
pip install skidl
```

**"kicad-cli not found"**
- Install KiCad 7+
- Add KiCad to system PATH

**"pcbnew module not found"**
- KiCad must be installed
- pcbnew is part of KiCad, not a separate package

**"FreeRouting can't route"**
- Check component placement (too tight?)
- Route critical traces manually first
- Increase board size if needed

See **QUICK_START.md** for detailed troubleshooting.

---

## Tools Research Summary

Based on extensive research of 2025 PCB design tools:

### ✓ Recommended Tools
1. **kicad-cli** - Excellent for exports (Gerbers, DRC)
2. **SKiDL** - Best for schematic/netlist generation
3. **pcbnew API** - Good for component placement
4. **FreeRouting** - Reliable autorouter with Python API
5. **kicad-python** - Wrapper for cross-version compatibility

### ⚠ Experimental Tools
1. **PCBflow** - Promising but alpha quality
2. **OrthoRoute** - New GPU-accelerated router

### ✗ Not Recommended
1. **PHDL** - Older, less active (use SKiDL instead)

See **PROGRAMMATIC_PCB_TOOLS.md** for detailed analysis.

---

## Can You Generate a PCB Fully Automatically?

### Short Answer: ~80% automated

**What CAN be automated:**
- ✓ Schematic/netlist generation (SKiDL)
- ✓ Component placement (pcbnew Python API)
- ✓ Autorouting (FreeRouting)
- ✓ DRC (kicad-cli)
- ✓ Gerber export (kicad-cli)

**What still needs human input:**
- ✗ Complex component placement optimization
- ✗ Critical trace routing (power, high-speed signals)
- ✗ Mechanical considerations (mounting holes, enclosures)
- ✗ Design verification and quality checks

**Realistic workflow:** Generate 80% automatically, manually refine the critical 20%

---

## Resources

### Official Documentation
- [KiCad Documentation](https://docs.kicad.org/)
- [SKiDL Documentation](https://devbisme.github.io/skidl/)
- [FreeRouting Docs](https://freerouting.org/)
- [KiCad Python API](https://dev-docs.kicad.org/en/apis-and-binding/pcbnew/)

### Community
- [KiCad Forum](https://forum.kicad.info/)
- [SKiDL GitHub](https://github.com/devbisme/skidl)
- [r/PrintedCircuitBoard](https://reddit.com/r/PrintedCircuitBoard)
- [EEVblog Forum](https://www.eevblog.com/forum/)

### Tutorials
- [Automating PCB Workflows](https://www.prasunbarua.com/2025/04/automating-pcb-design-workflows-with.html)
- [Programmatic Layout Tutorial](https://jeffmcbride.net/programmatic-layout-with-kicad-and-python/)
- [SKiDL on Hackaday](https://hackaday.com/2016/12/28/skidl-script-your-circuits-in-python/)

### Example Projects
- [SKiDL Examples](https://github.com/devbisme/skidl/tree/master/examples)
- [PCBflow Examples](https://github.com/michaelgale/pcbflow/tree/main/examples)

---

## Contributing

Found an issue or have improvements?
1. Test your changes
2. Update relevant documentation
3. Submit changes

---

## License

This project is part of the ESP32 Datalogger v2 project.
See main repository for license information.

---

## Changelog

### 2025-12-21
- Initial research on programmatic PCB design tools
- Created comprehensive documentation
- Implemented SKiDL circuit definition
- Added automation scripts for routing and export
- Created quick start guide

---

**Ready to build your PCB?** Start with [QUICK_START.md](QUICK_START.md)!

**Want to understand the tools?** Read [PROGRAMMATIC_PCB_TOOLS.md](PROGRAMMATIC_PCB_TOOLS.md)!

**Need circuit details?** Check [pnan.md](pnan.md)!
