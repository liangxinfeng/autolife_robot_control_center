# Scoped reverse docking correction

`main.cpp`, `docking_server.cpp`, `dock_database.cpp`, `navigator.cpp` originate
from ros-navigation/navigation2 tag **1.3.10**, directory
`nav2_docking/opennav_docking/src`. Original Apache-2.0 notices are retained.
https://github.com/ros-navigation/navigation2/tree/1.3.10/nav2_docking/opennav_docking

Only algorithm change: `approachDock` projects the target 0.25 m in the ORIGINAL
dock frame, then flips heading for backward driving, matching the newer Jazzy
ordering. The old order projected to 0.25 m BEFORE contact and made the smooth
controller singular there. See https://github.com/ros-navigation/navigation2/issues/6073
and https://api.nav2.org/nav2-jazzy/html/docking__server_8cpp_source.html

`dock_projection.hpp` isolates that geometry for regression testing. Compile
these sources into `control_center_navigation/docking_server_compat`, reusing
installed headers, controller and charging plugin; do not replace /opt/ros files
or override the vendor's opennav_docking package. Installed API baseline: 1.3.10.
Collision checks, retries, charging confirmation and cancellation remain intact.

2026-09-09 observed 307 failure: remaining distance 0.255 m, angular command
alternating +/-0.12 rad/s, reverse velocity tending to zero. Manager stopped at
8.6-degree heading error after 11.2 s. This was NOT approach timeout or charging
success. Drift protection remains unchanged.
