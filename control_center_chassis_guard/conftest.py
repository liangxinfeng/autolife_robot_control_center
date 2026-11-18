"""Test-suite bootstrap.

Keeps ``control_center_chassis_guard`` importable straight from the source tree
(so the suite does not depend on the workspace being installed and sourced) and
makes ``fake_redis`` importable regardless of the working directory.

The UI integration tests still need a sourced ROS 2 workspace and Qt; they skip
themselves with a reason when that is not available.
"""

import sys
from pathlib import Path

_PACKAGE_ROOT = Path(__file__).resolve().parent
for _candidate in (_PACKAGE_ROOT, _PACKAGE_ROOT / 'test'):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))
