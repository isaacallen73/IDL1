# Quick Start Guide: Programmatic PCB Design

This guide shows you how to generate your datalogger PCB using the programmatic tools researched.

---

## Prerequisites

### 1. Install KiCad 8 or 9
```bash
# Download from: https://www.kicad.org/download/

# Or on Linux:
sudo apt install kicad

# Verify installation:
kicad-cli --version
```

### 2. Install Python Tools
```bash
# Create virtual environment
python -m venv pcb-env

# Activate (Linux/Mac)
source pcb-env/bin/activate

# Activate (Windows)
pcb-env\Scripts\activate

# Install required packages
pip install skidl
```

### 3. Optional: Install kicad-python
```bash
pip install kicad-python
```

---

## Quick Start: 3 Options

### Option A: Fully Automated (Experimental)
**Best for:** Simple boards, learning, rapid prototyping
**Automation:** ~80%
**Quality:** Good

```bash
# Generate circuit and netlist
python generate_pcb.py

# Open in KiCad
kicad output/datalogger.kicad_pcb

# Use FreeRouting plugin
# Tools → External Plugins → FreeRouting

# Export Gerbers
python autoroute_example.py export output/datalogger.kicad_pcb
```

---

### Option B: Semi-Automated (Recommended)
**Best for:** Production boards, best quality
**Automation:** ~40%
**Quality:** Excellent

```bash
# Step 1: Generate netlist with SKiDL
python generate_pcb.py

# Step 2: Create new PCB in KiCad
# - Open KiCad
# - Create new project
# - Open PCB Editor

# Step 3: Import netlist
# - Tools → Load Netlist
# - Select: output/datalogger.net

# Step 4: Place components
# Option A: Use placement script
#   - Open KiCad Scripting Console (Tools → Scripting Console)
#   - Run: exec(open('output/place_components.py').read())

# Option B: Manual placement (recommended for first time)
#   - Arrange components according to pnan.md specifications
#   - XIAO near center
#   - GPS opposite corner
#   - BMI160 near XIAO
#   - SD card at accessible edge

# Step 5: Route board
# Option A: FreeRouting plugin
#   - Tools → External Plugins → FreeRouting
#   - Let it autoroute
#   - Manual cleanup

# Option B: Manual routing (best quality)
#   - Route critical traces first (GPS, power)
#   - Add ground plane
#   - Route remaining signals

# Step 6: DRC and export
python autoroute_example.py export output/datalogger.kicad_pcb
```

---

### Option C: Manual with Script Assistance
**Best for:** Learning KiCad, complex boards
**Automation:** ~20%
**Quality:** Excellent

1. **Use SKiDL for netlist only**
   ```bash
   python generate_pcb.py
   ```

2. **Do everything else in KiCad GUI**
   - Import netlist
   - Manual placement
   - Manual routing
   - Manual Gerber export

3. **Refer to pnan.md for specifications**

---

## Step-by-Step Walkthrough (Option B - Recommended)

### Step 1: Generate Circuit Netlist (2 minutes)

```bash
# Review and customize the circuit
# Edit generate_pcb.py if needed to adjust components

# Generate netlist
python generate_pcb.py

# You should see:
# - output/datalogger.net (netlist file)
# - output/place_components.py (placement script)
```

**What this does:**
- Defines complete circuit in Python (SKiDL)
- Generates KiCad-compatible netlist
- Creates component placement helper script

---

### Step 2: Create KiCad Project (2 minutes)

1. Open KiCad
2. File → New Project
3. Name: "Datalogger"
4. Save in: `PCB Design/kicad/`

---

### Step 3: Import Netlist (1 minute)

1. Open PCB Editor (`.kicad_pcb` file)
2. Tools → Load Netlist
3. Browse to: `output/datalogger.net`
4. Click "Update PCB"
5. All components appear in a pile

---

### Step 4: Place Components (10-30 minutes)

**Option A: Semi-Automated Placement**

1. Tools → Scripting Console
2. Run:
   ```python
   exec(open('output/place_components.py').read())
   ```
3. Components jump to approximate positions
4. Fine-tune manually

**Option B: Manual Placement (Recommended for First Time)**

Follow the guidelines from `pnan.md`:

```
Board: ~60mm x 40mm (adjust to battery size)

Component Layout:
┌─────────────────────────────────────┐
│  [Front Conn]         [Rear Conn]   │
│                                      │
│  [BMI160]  [XIAO]         [GPS]     │
│            ESP32-C6       Module    │
│                                      │
│  [SD Card]        [Battery]         │
│                   Connector         │
│                                      │
│         [Handlebar Conn]             │
└─────────────────────────────────────┘
```

**Placement Checklist:**
- [ ] XIAO ESP32-C6: Center or near center (USB access!)
- [ ] GPS Module: Opposite corner from XIAO
- [ ] BMI160: Near XIAO, away from GPS
- [ ] SD Card: Accessible edge for insertion
- [ ] Connectors: Opposite edges or labeled clearly
- [ ] Battery Connector: Near XIAO BAT pads
- [ ] Decoupling Caps: Within 2-3mm of IC power pins

---

### Step 5: Add Board Outline (5 minutes)

1. Select "Edge.Cuts" layer
2. Draw board outline (rectangle or rounded)
3. Typical size: 60mm x 40mm
4. Add mounting holes if needed (3mm diameter)

---

### Step 6: Create Ground Plane (5 minutes)

1. Select "B.Cu" (bottom copper) layer
2. Add → Filled Zone
3. Net: GND
4. Draw zone around entire board
5. Right-click → Fill All Zones

Repeat for top layer if desired (partial fills OK)

---

### Step 7: Route Critical Traces (15-60 minutes)

**Priority 1: Power Rails**
- VBAT → XIAO BAT+
- 3V3 → All ICs (wide traces, 0.5mm)
- GND → All ICs (use ground plane)

**Priority 2: GPS Traces**
- Keep GPS TX/RX traces short
- Keep antenna area clear of copper
- No ground pour under GPS antenna

**Priority 3: SPI Bus**
- Keep SCK, MOSI, MISO roughly equal length
- Run parallel where possible
- Ground trace nearby

**Priority 4: Everything Else**
- Analog traces (pressure sensors) away from digital
- Route remaining GPIOs

---

### Step 8: Autoroute Remaining Traces (Optional)

If you don't want to route everything manually:

1. Route critical traces manually (power, GPS)
2. Tools → External Plugins → FreeRouting
3. Let FreeRouting route remaining traces
4. Review and adjust

**Or:**

```bash
python autoroute_example.py route output/datalogger.kicad_pcb
```

---

### Step 9: Design Rule Check (5 minutes)

1. Inspect → Design Rules Checker
2. Click "Run DRC"
3. Review errors/warnings
4. Fix issues:
   - Clearance violations
   - Unconnected traces
   - Copper too close to board edge

**Or via CLI:**

```bash
kicad-cli pcb drc --output drc_report.txt output/datalogger.kicad_pcb
```

---

### Step 10: Add Silkscreen (10 minutes)

1. Select "F.Silkscreen" layer
2. Add text labels:
   - Component references (U1, J1, etc.)
   - Connector pin numbers
   - Board name and version
   - Polarity markings (+/- for battery)
3. Ensure text is readable (min 1mm height)

---

### Step 11: Final Checks (10 minutes)

- [ ] USB port accessible
- [ ] SD card accessible
- [ ] All connectors accessible
- [ ] GPS antenna area clear
- [ ] Ground plane filled
- [ ] No DRC errors
- [ ] Silkscreen clear and readable
- [ ] Board outline correct size

---

### Step 12: Export Gerbers (2 minutes)

**Option A: CLI (Automated)**

```bash
python autoroute_example.py export output/datalogger.kicad_pcb
```

This exports:
- Gerber files (all layers)
- Drill files
- Position file (CSV)
- 3D STEP model

**Option B: GUI**

1. File → Fabrication Outputs → Gerbers
2. Select layers:
   - F.Cu, B.Cu (copper)
   - F.Paste, B.Paste (solder paste)
   - F.Silkscreen, B.Silkscreen (labels)
   - F.Mask, B.Mask (solder mask)
   - Edge.Cuts (board outline)
3. Click "Plot"
4. Click "Generate Drill Files"

---

### Step 13: Order PCBs (10 minutes)

1. Zip the Gerber files
2. Upload to PCB manufacturer:
   - **JLCPCB** (cheap, fast): https://jlcpcb.com
   - **PCBWay**: https://www.pcbway.com
   - **OSH Park** (USA): https://oshpark.com

3. Select options:
   - Layers: 2
   - Thickness: 1.6mm
   - Color: Green (or your choice)
   - Surface finish: HASL or ENIG
   - Quantity: 5-10 (minimum order)

4. Price: ~$5-20 for 5 boards
5. Shipping: 1-2 weeks

---

## Troubleshooting

### "Module 'pcbnew' not found"

**Solution:** KiCad Python bindings require KiCad to be installed. The `pcbnew` module is part of KiCad, not a separate pip package.

**Workaround:** Use manual placement instead of Python script.

---

### "kicad-cli: command not found"

**Solution:** Add KiCad to your system PATH.

**Linux/Mac:**
```bash
export PATH="/Applications/KiCad/KiCad.app/Contents/MacOS:$PATH"
```

**Windows:**
```powershell
# Add to PATH: C:\Program Files\KiCad\bin
```

---

### SKiDL generates errors about missing footprints

**Solution:** Use generic footprints or create custom ones.

**Fix in generate_pcb.py:**
```python
# Change:
footprint='Package_LGA:LGA-14_3x2.5mm_P0.5mm'

# To:
footprint='Package_SO:SOIC-8_3.9x4.9mm_P1.27mm'  # Generic alternative
```

---

### FreeRouting can't route board

**Possible causes:**
- Components too close together
- Design rules too strict
- Board too small for components

**Solutions:**
1. Increase board size
2. Adjust component placement
3. Relax design rules temporarily
4. Route critical traces manually first

---

### DRC shows clearance violations

**Solution:**
1. Increase trace clearance in design rules
2. Move components further apart
3. Adjust copper pour clearances

---

## Next Steps After PCB Arrives

1. **Visual Inspection**
   - Check for manufacturing defects
   - Verify board dimensions
   - Check silkscreen alignment

2. **Continuity Testing**
   - Test ground plane continuity
   - Test power rails (no shorts!)
   - Test connector pins

3. **Assembly**
   - Solder components (smallest first)
   - Double-check polarity (ICs, LEDs, connectors)
   - Use flux and good soldering iron

4. **Testing**
   - Power on with current-limited supply
   - Check 3.3V rail
   - Upload firmware
   - Test each module (GPS, SD, IMU, etc.)

---

## Resources

### Files in This Directory
- `PROGRAMMATIC_PCB_TOOLS.md` - Detailed tool research
- `generate_pcb.py` - Circuit definition and netlist generator
- `autoroute_example.py` - Automation and export scripts
- `pnan.md` - Complete PCB specification
- `KICAD_WORKFLOW.md` - Manual KiCad workflow
- `DESIGN_CHECKLIST.md` - Design verification checklist

### External Resources
- [KiCad Documentation](https://docs.kicad.org/)
- [SKiDL Documentation](https://devbisme.github.io/skidl/)
- [KiCad Forum](https://forum.kicad.info/)
- [FreeRouting](https://github.com/freerouting/freerouting)

### Example Projects
- [SKiDL Examples](https://github.com/devbisme/skidl/tree/master/examples)
- [PCBflow Examples](https://github.com/michaelgale/pcbflow/tree/main/examples)

---

## Summary: Recommended Workflow

For the **highest quality** datalogger PCB:

1. **Schematic**: SKiDL (automated) → `python generate_pcb.py`
2. **Netlist Import**: KiCad GUI
3. **Placement**: Manual with reference to `pnan.md`
4. **Routing**: Manual (critical traces) + FreeRouting (remaining)
5. **DRC**: kicad-cli (automated)
6. **Export**: kicad-cli (automated) → `python autoroute_example.py export`

**Total time:** 2-4 hours for first iteration

**Automation:** ~40% automated, 60% manual (where it matters most)

---

**Ready to start?** Run `python generate_pcb.py` and follow the steps above!

**Questions?** Check `PROGRAMMATIC_PCB_TOOLS.md` for detailed tool documentation.

**Need help?** Review `pnan.md` for complete circuit specifications.
