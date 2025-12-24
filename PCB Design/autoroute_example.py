#!/usr/bin/env python3
"""
Automated Routing Example using FreeRouting and KiCad
Demonstrates complete workflow from placement to Gerber export

This script shows how to integrate FreeRouting autorouter with KiCad Python API
"""

import subprocess
import os
import sys
import json
from pathlib import Path


class PCBAutomation:
    """
    Automates PCB routing and export workflow
    """

    def __init__(self, pcb_file, output_dir='output'):
        self.pcb_file = pcb_file
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

    def export_dsn(self):
        """
        Export DSN file for FreeRouting
        DSN is the Specctra format that FreeRouting uses
        """
        print("Exporting DSN file for FreeRouting...")

        dsn_file = self.output_dir / f"{Path(self.pcb_file).stem}.dsn"

        # Method 1: Use kicad-cli (if available in KiCad 8+)
        # Note: As of KiCad 8, DSN export might need to be done via GUI or Python API

        # Method 2: Use pcbnew Python API
        try:
            import pcbnew

            board = pcbnew.LoadBoard(self.pcb_file)

            # Export DSN via Python
            # Note: This requires proper setup of export options
            print(f"Loaded board: {self.pcb_file}")
            print(f"DSN file would be saved to: {dsn_file}")

            # The actual DSN export in pcbnew is complex
            # For now, use FreeRouting KiCad plugin which handles this automatically

            print("\nNote: DSN export is typically done through KiCad GUI:")
            print("  File → Export → Specctra DSN")
            print("  Or use FreeRouting plugin: Tools → External Plugins → FreeRouting")

        except ImportError:
            print("Error: pcbnew module not found. Make sure KiCad is installed.")
            print("\nAlternative: Export DSN manually from KiCad:")
            print("  1. Open PCB in KiCad")
            print("  2. File → Export → Specctra DSN")
            print(f"  3. Save to: {dsn_file}")

        return dsn_file

    def run_freerouting(self, dsn_file, ses_file=None):
        """
        Run FreeRouting autorouter

        Args:
            dsn_file: Input Specctra DSN file
            ses_file: Output session file (if None, uses default)
        """
        if ses_file is None:
            ses_file = self.output_dir / f"{Path(dsn_file).stem}.ses"

        print(f"\nRunning FreeRouting on {dsn_file}...")

        # Check if FreeRouting is installed
        freerouting_jar = self._find_freerouting()

        if not freerouting_jar:
            print("Error: FreeRouting not found!")
            print("\nInstall FreeRouuting:")
            print("  1. Download from: https://github.com/freerouting/freerouting/releases")
            print("  2. Or install as KiCad plugin via Plugin Manager")
            print("  3. Or use KiCad GUI: Tools → External Plugins → FreeRouting")
            return None

        # Run FreeRouting
        cmd = [
            'java', '-jar', freerouting_jar,
            '-de', str(dsn_file),  # Input DSN
            '-do', str(ses_file),  # Output SES
            '-mp', '20',           # Max passes
            '-mt', 'auto',         # Threads
        ]

        print(f"Command: {' '.join(cmd)}")

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=300  # 5 minute timeout
            )

            if result.returncode == 0:
                print(f"✓ Routing complete! Output: {ses_file}")
                return ses_file
            else:
                print(f"✗ Routing failed!")
                print(f"Error: {result.stderr}")
                return None

        except subprocess.TimeoutExpired:
            print("✗ Routing timed out after 5 minutes")
            return None
        except FileNotFoundError:
            print("✗ Java not found. Install Java Runtime Environment (JRE)")
            return None

    def _find_freerouting(self):
        """
        Try to locate FreeRouting JAR file
        """
        # Common locations
        possible_paths = [
            'freerouting.jar',
            './freerouting/freerouting.jar',
            os.path.expanduser('~/freerouting/freerouting.jar'),
            'C:/Program Files/KiCad/bin/freerouting.jar',
        ]

        for path in possible_paths:
            if os.path.exists(path):
                return path

        return None

    def import_ses(self, ses_file):
        """
        Import SES (session) file back into KiCad PCB
        """
        print(f"\nImporting routing from {ses_file}...")

        # This is typically done through KiCad GUI
        print("Import SES file in KiCad:")
        print("  1. Open PCB in KiCad")
        print("  2. File → Import → Specctra Session")
        print(f"  3. Select: {ses_file}")

        # Alternatively, use pcbnew Python API (more complex)
        try:
            import pcbnew

            board = pcbnew.LoadBoard(self.pcb_file)

            # Import session file
            # Note: This requires calling the appropriate import function
            # The exact API varies by KiCad version

            print(f"✓ Would import {ses_file} to {self.pcb_file}")

        except ImportError:
            pass

    def run_drc(self):
        """
        Run Design Rule Check using kicad-cli
        """
        print("\nRunning Design Rule Check...")

        drc_report = self.output_dir / 'drc_report.txt'

        cmd = [
            'kicad-cli', 'pcb', 'drc',
            '--output', str(drc_report),
            self.pcb_file
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                print(f"✓ DRC complete! Report: {drc_report}")

                # Check if there are errors
                with open(drc_report, 'r') as f:
                    report = f.read()
                    if 'error' in report.lower():
                        print("⚠ DRC errors found! Review the report.")
                    else:
                        print("✓ No DRC errors!")

                return drc_report
            else:
                print(f"✗ DRC failed: {result.stderr}")
                return None

        except FileNotFoundError:
            print("✗ kicad-cli not found. Install KiCad 7+ and add to PATH")
            return None

    def export_gerbers(self):
        """
        Export Gerber files using kicad-cli
        """
        print("\nExporting Gerber files...")

        gerber_dir = self.output_dir / 'gerbers'
        gerber_dir.mkdir(exist_ok=True)

        # Export Gerbers
        cmd = [
            'kicad-cli', 'pcb', 'export', 'gerbers',
            '--output', str(gerber_dir),
            self.pcb_file
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                print(f"✓ Gerbers exported to: {gerber_dir}")
            else:
                print(f"✗ Gerber export failed: {result.stderr}")
                return None

        except FileNotFoundError:
            print("✗ kicad-cli not found")
            return None

        # Export drill files
        cmd = [
            'kicad-cli', 'pcb', 'export', 'drill',
            '--output', str(gerber_dir),
            self.pcb_file
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                print(f"✓ Drill files exported to: {gerber_dir}")
                return gerber_dir
            else:
                print(f"✗ Drill export failed: {result.stderr}")
                return None

        except FileNotFoundError:
            return None

    def export_position_file(self):
        """
        Export component position file for assembly
        """
        print("\nExporting position file...")

        pos_file = self.output_dir / f"{Path(self.pcb_file).stem}_pos.csv"

        cmd = [
            'kicad-cli', 'pcb', 'export', 'pos',
            '--output', str(pos_file),
            '--format', 'csv',
            '--units', 'mm',
            '--side', 'both',
            self.pcb_file
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                print(f"✓ Position file exported: {pos_file}")
                return pos_file
            else:
                print(f"✗ Position export failed: {result.stderr}")
                return None

        except FileNotFoundError:
            print("✗ kicad-cli not found")
            return None

    def export_3d_model(self):
        """
        Export 3D STEP model for mechanical design
        """
        print("\nExporting 3D STEP model...")

        step_file = self.output_dir / f"{Path(self.pcb_file).stem}.step"

        cmd = [
            'kicad-cli', 'pcb', 'export', 'step',
            '--output', str(step_file),
            self.pcb_file
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                print(f"✓ STEP model exported: {step_file}")
                return step_file
            else:
                print(f"✗ STEP export failed: {result.stderr}")
                return None

        except FileNotFoundError:
            print("✗ kicad-cli not found")
            return None


def complete_workflow(pcb_file):
    """
    Run complete automated workflow
    """
    print("=" * 70)
    print("PCB Automation Workflow")
    print("=" * 70)

    automation = PCBAutomation(pcb_file)

    # Step 1: Export DSN for routing
    print("\n[Step 1] Export DSN file")
    dsn_file = automation.export_dsn()

    # Step 2: Run autorouter (FreeRouting)
    print("\n[Step 2] Run FreeRouting autorouter")
    print("NOTE: This step typically requires using KiCad plugin")
    print("      Tools → External Plugins → FreeRouting")
    # ses_file = automation.run_freerouting(dsn_file)

    # Step 3: Import routing results
    # if ses_file:
    #     print("\n[Step 3] Import routing results")
    #     automation.import_ses(ses_file)

    # Step 4: Run DRC
    print("\n[Step 4] Run Design Rule Check")
    drc_report = automation.run_drc()

    # Step 5: Export Gerbers
    if drc_report:
        print("\n[Step 5] Export Gerbers")
        gerber_dir = automation.export_gerbers()

        # Step 6: Export position file
        print("\n[Step 6] Export position file")
        automation.export_position_file()

        # Step 7: Export 3D model
        print("\n[Step 7] Export 3D STEP model")
        automation.export_3d_model()

    print("\n" + "=" * 70)
    print("Workflow Complete!")
    print("=" * 70)


def alternative_freerouting_workflow():
    """
    Show alternative workflow using FreeRouting KiCad plugin
    This is more reliable than calling FreeRouting JAR directly
    """

    workflow = """
    === RECOMMENDED WORKFLOW: FreeRouting KiCad Plugin ===

    1. Open your PCB in KiCad PCBNew
       - File → Open → datalogger.kicad_pcb

    2. Ensure all components are placed
       - Use the placement script: place_components.py
       - Or place manually

    3. Install FreeRouting Plugin (if not already installed)
       - Tools → Plugin and Content Manager
       - Search for "FreeRouting"
       - Install

    4. Run FreeRouting
       - Tools → External Plugins → FreeRouting
       - This will:
         * Export DSN automatically
         * Launch FreeRouting
         * Route the board
         * Import results back

    5. Review routing
       - Check for any unrouted nets
       - Verify critical traces (GPS, power, etc.)
       - Manual touch-up if needed

    6. Run DRC
       - Inspect → Design Rules Checker
       - Fix any violations

    7. Export Gerbers (automated)
       - Run this script: python autoroute_example.py export <pcb_file>
       - Or use KiCad: File → Fabrication Outputs → Gerbers

    === FULLY AUTOMATED WITH PYTHON ===

    For full automation, use:
       1. SKiDL for circuit definition
       2. pcbnew Python API for placement
       3. FreeRouting CLI (requires Java)
       4. kicad-cli for exports

    See PROGRAMMATIC_PCB_TOOLS.md for details.
    """

    print(workflow)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description='Automate PCB routing and export workflow'
    )

    parser.add_argument(
        'command',
        choices=['route', 'export', 'workflow', 'help'],
        help='Command to run'
    )

    parser.add_argument(
        'pcb_file',
        nargs='?',
        help='Path to KiCad PCB file (.kicad_pcb)'
    )

    args = parser.parse_args()

    if args.command == 'help':
        alternative_freerouting_workflow()

    elif args.command == 'export':
        if not args.pcb_file:
            print("Error: PCB file required for export command")
            sys.exit(1)

        automation = PCBAutomation(args.pcb_file)
        automation.run_drc()
        automation.export_gerbers()
        automation.export_position_file()
        automation.export_3d_model()

    elif args.command == 'route':
        if not args.pcb_file:
            print("Error: PCB file required for route command")
            sys.exit(1)

        automation = PCBAutomation(args.pcb_file)
        dsn_file = automation.export_dsn()
        automation.run_freerouting(dsn_file)

    elif args.command == 'workflow':
        if not args.pcb_file:
            print("Error: PCB file required for workflow command")
            sys.exit(1)

        complete_workflow(args.pcb_file)

    else:
        alternative_freerouting_workflow()
