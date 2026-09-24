#!/usr/bin/env python3
"""Deprecated wrapper — use src/pythontool/generate_field_waypoints.py instead."""

import subprocess
import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[4]
SCRIPT = WORKSPACE / "src/pythontool/generate_field_waypoints.py"

if __name__ == "__main__":
    raise SystemExit(
        subprocess.call([sys.executable, str(SCRIPT), "--viz", *sys.argv[1:]])
    )
