"""
Simple device discovery and factory for temperature controllers.
"""

from serial.tools import list_ports
from typing import Dict, Tuple, Type, Optional, List

from .mtd1020t import MTD1020T
from .covesion_oc3 import OC3

# Registry mapping USB VID:PID to controller classes
# Add your device VID:PID pairs here
CONTROLLER_REGISTRY: Dict[Tuple[int, int], Type] = {
    (0x1313, 0x80C9): MTD1020T,  # MTD1020T VID:PID
    (0x0403, 0x6001): OC3,
}


def discover() -> List[Tuple[str, str, Optional[str]]]:
    """Discover connected temperature controllers.

    Scans all serial ports and matches VID:PID against the registry.

    Returns:
        List of tuples: (port_path, description, controller_type_name)
        Empty list if no devices found.

    Example:
        >>> devices = discover()
        >>> for port, desc, ctrl_type in devices:
        ...     print(f'{port}: [{ctrl_type}] {desc}')
        /dev/ttyUSB0: [MTD1020T] FTDI USB Serial Converter
    """
    ports = list_ports.comports()
    device_list: List[Tuple[str, str, Optional[str]]] = []

    for port in ports:
        # Skip ports without VID:PID (virtual ports, etc.)
        if port.vid is None or port.pid is None:
            continue

        current_id = (port.vid, port.pid)

        # Check if this VID:PID is in our registry
        if current_id in CONTROLLER_REGISTRY:
            controller_type = CONTROLLER_REGISTRY[current_id]
            device_list.append((
                port.device,  # Port path (e.g., '/dev/ttyUSB0')
                port.description or 'Unknown',  # Human-readable description
                controller_type.__name__,  # Controller class name (e.g., 'MTD1020T')
            ))

    return device_list


def create_controller(port: str):
    """Factory function to create appropriate controller based on VID:PID.

    Args:
        port: Serial port path (e.g., '/dev/ttyUSB0', 'COM3').

    Returns:
        Controller instance of the appropriate type.

    Raises:
        ValueError: If no matching controller found for this port.

    Example:
        >>> controller = create_controller('/dev/ttyUSB0')
        >>> print(controller.get_firmware_version())
    """
    # Get all ports
    ports = list_ports.comports()

    # Find the port info for the requested port
    port_info = None
    for p in ports:
        if p.device == port:
            port_info = p
            break

    if not port_info:
        raise ValueError(f'Port {port} not found')

    if port_info.vid is None or port_info.pid is None:
        raise ValueError(f'Port {port} does not have VID:PID information')

    # Look up controller type
    current_id = (port_info.vid, port_info.pid)

    if current_id not in CONTROLLER_REGISTRY:
        raise ValueError(
            f'Unknown device at {port} (VID:PID = {port_info.vid:04X}:{port_info.pid:04X}). '
            f'Add this device to CONTROLLER_REGISTRY.'
        )

    controller_class = CONTROLLER_REGISTRY[current_id]
    return controller_class(port=port)


def main():
    """CLI tool to discover devices."""
    devices = discover()

    if not devices:
        print('No devices found')
        print('\nTo find your device VID:PID, run:')
        print('  python -m serial.tools.list_ports -v')
        return

    print('Discovered devices:')
    print('-' * 70)
    for port, description, controller_type in devices:
        print(f'{port:20} -> [{controller_type:12}] {description}')
    print('-' * 70)
    print(f'Total: {len(devices)} device(s)')


if __name__ == '__main__':
    main()
