from serial.tools import list_ports
from typing import Dict, Tuple

CONTROLLER_REGISTRY: Dict[Tuple[int, int], str] = {
    (0x1313, 0x80C9): 'MTD',
}


ports = list_ports.comports()

for p in ports:
    if p.pid is None:
        continue
    if p.vid is None:
        continue

    id = (p.vid, p.pid)
    print(id, CONTROLLER_REGISTRY[id], p.device)
