#!/usr/bin/env python3
"""Wait for the world generator to finish writing its output."""
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

world = Path(sys.argv[1])
deadline = time.monotonic() + 45
while time.monotonic() < deadline:
    try:
        tree = ET.parse(world)
        if len(tree.findall('.//actor')) == int(sys.argv[2]):
            print(f'Generated world ready: {world}', flush=True)
            sys.exit(0)
    except (OSError, ET.ParseError):
        pass
    time.sleep(0.2)
sys.exit('Timed out waiting for the generated world')
