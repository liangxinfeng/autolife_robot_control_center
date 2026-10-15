"""One naming rule shared by dispatch, docking approach and departure."""
import re
import unicodedata

def is_charger_waypoint(name, primary='充电桩'):
    name = unicodedata.normalize('NFKC', str(name)).strip()
    primary = unicodedata.normalize('NFKC', str(primary)).strip()
    prefixes = {primary, '充电桩', '充电装'}
    return any(prefix and re.fullmatch(re.escape(prefix) + r'(?:\s*[0-9]+)?', name)
               for prefix in prefixes)
