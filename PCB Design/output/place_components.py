#!/usr/bin/env python3
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
        print(f"OK Placed {ref} at ({x}, {y})")
    else:
        print(f"WARNING Component {ref} not found")

pcbnew.Refresh()
print("\nPlacement complete! Now route the board.")
