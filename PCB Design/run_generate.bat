@echo off
REM Set KiCad environment variables before running Python
SET KICAD9_SYMBOL_DIR=C:\Program Files\KiCad\9.0\share\kicad\symbols
SET KICAD9_FOOTPRINT_DIR=C:\Program Files\KiCad\9.0\share\kicad\footprints

echo Running PCB generation with KiCad 9 libraries...
python generate_pcb_fixed.py

pause
