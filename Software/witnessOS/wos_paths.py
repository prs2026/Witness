"""Put the HITL and Horizon tool folders on sys.path (import this first).

witnessOS reuses those tools' modules by import. Neither folder is a
package, so their directories are added to the module search path, exactly
as each tool does for itself when run as a script.
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SOFTWARE = os.path.dirname(HERE)
HITL_DIR = os.path.join(SOFTWARE, "HITL")
HORIZON_DIR = os.path.join(SOFTWARE, "Horizon")

for _path in (HERE, HITL_DIR, HORIZON_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)
