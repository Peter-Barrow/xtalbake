"""
MTD1020T TEC Controller Serial Interface
Implements serial communication and command groups for the MTD1020T device.
"""

import serial
import time
from typing import Tuple, Dict, Optional, List, Literal
from enum import IntFlag

__all__ = [
    'MTD1020T',
]


class ErrorFlags(IntFlag):
    """Error register bit flags for MTD1020T.

    Attributes:
        ENABLE_PIN_NOT_SET: Enable pin not set to L (GND)
        INTERNAL_TEMP_HIGH: Internal temperature too high
        THERMAL_LATCH_UP: TEC current at limit without temperature improvement
        CYCLING_TIME_SMALL: Cycling time too small
        NO_SENSOR: No Sensor detected
        NO_TEC: No TEC detected (connection open)
        TEC_MISPOLED: TEC mispoled
        VALUE_OUT_OF_RANGE: Value out of range
        INVALID_COMMAND: Invalid command
    """

    ENABLE_PIN_NOT_SET = 1 << 0  # Bit 0
    INTERNAL_TEMP_HIGH = 1 << 1  # Bit 1
    THERMAL_LATCH_UP = 1 << 2  # Bit 2
    CYCLING_TIME_SMALL = 1 << 3  # Bit 3
    NO_SENSOR = 1 << 4  # Bit 4
    NO_TEC = 1 << 5  # Bit 5
    TEC_MISPOLED = 1 << 6  # Bit 6
    VALUE_OUT_OF_RANGE = 1 << 13  # Bit 13
    INVALID_COMMAND = 1 << 14  # Bit 14


def write_command(conn: serial.Serial, command: str) -> None:
    """Write a command to the MTD1020T device.

    Args:
        conn: Serial connection object
        command: Command string (e.g., "L?" or "Lx1500")
    """
    # Add line feed terminator and encode
    cmd_bytes = f'{command}\n'.encode('ascii')
    _ = conn.write(cmd_bytes)
    conn.flush()


def read_response(conn: serial.Serial, timeout: float = 2.0) -> str:
    """Read response from MTD1020T device until line feed terminator.

    Args:
        conn: Serial connection object
        timeout: Read timeout in seconds

    Returns:
        Response string with brackets and whitespace stripped
    """
    original_timeout = conn.timeout
    conn.timeout = timeout

    response = conn.read_until(b'\n')
    conn.flush()

    conn.timeout = original_timeout

    return response.decode('utf-8').strip()


def query(conn: serial.Serial, command: str, timeout: float = 2.0) -> str:
    """Send a command and read the response.

    Args:
        conn: Serial connection object
        command: Command string
        timeout: Read timeout in seconds

    Returns:
        Response string
    """
    write_command(conn, command)
    response = read_response(conn, timeout)
    if response == b'':
        time.sleep(0.1)
        write_command(conn, command)
        response = read_response(conn, timeout)
    return response


def sys_info_get_version(conn: serial.Serial) -> str:
    """Read hardware and software version.

    Args:
        conn: Serial connection object

    Returns:
        Version string (e.g., "MTD1020 FW0.6.8")
    """
    return query(conn, 'm?')


def sys_info_get_uuid(conn: serial.Serial) -> str:
    """Read UUID (Universal Unique Identifier).
    Args:
        conn: Serial connection object

    Returns:
        32-character hexadecimal UUID string
    """
    return query(conn, 'u?')


def sys_info_get_error_register(conn: serial.Serial) -> int:
    """Read Error Register as raw integer value.
    Args:
        conn: Serial connection object

    Returns:
        16-bit error register value
    """
    response = query(conn, 'E?')
    return int(response)


def sys_info_get_error_flags(conn: serial.Serial) -> ErrorFlags:
    """Read Error Register and return as ErrorFlags enum.
    Args:
        conn: Serial connection object

    Returns:
        ErrorFlags enum with active error bits set
    """
    error_value = sys_info_get_error_register(conn)
    return ErrorFlags(error_value)


def sys_info_get_active_errors(conn: serial.Serial) -> Dict[str, bool]:
    """Read Error Register and return dictionary of active errors.
    Args:
        conn: Serial connection object

    Returns:
        Dictionary mapping error names to boolean values indicating
        whether each error is active

    Example:
        >>> errors = system.get_active_errors()
        >>> if errors['INTERNAL_TEMP_HIGH']:
        ...     print('Device temperature too high!')
    """
    error_flags = sys_info_get_error_flags(conn)
    return {
        'ENABLE_PIN_NOT_SET': bool(
            error_flags & ErrorFlags.ENABLE_PIN_NOT_SET,
        ),
        'INTERNAL_TEMP_HIGH': bool(
            error_flags & ErrorFlags.INTERNAL_TEMP_HIGH,
        ),
        'THERMAL_LATCH_UP': bool(
            error_flags & ErrorFlags.THERMAL_LATCH_UP,
        ),
        'CYCLING_TIME_SMALL': bool(
            error_flags & ErrorFlags.CYCLING_TIME_SMALL,
        ),
        'NO_SENSOR': bool(error_flags & ErrorFlags.NO_SENSOR),
        'NO_TEC': bool(error_flags & ErrorFlags.NO_TEC),
        'TEC_MISPOLED': bool(error_flags & ErrorFlags.TEC_MISPOLED),
        'VALUE_OUT_OF_RANGE': bool(
            error_flags & ErrorFlags.VALUE_OUT_OF_RANGE,
        ),
        'INVALID_COMMAND': bool(error_flags & ErrorFlags.INVALID_COMMAND),
    }


def sys_info_reset_error_register(conn: serial.Serial) -> None:
    """Reset the Error Register.
    Args:
        conn: Serial connection object

    Note:
        Can also be reset by toggling the Enable pin Off and On again.
    """
    write_command(conn, 'c')


def safety_bitmask_set_masked_errors(
    conn: serial.Serial, *errors: ErrorFlags
) -> None:
    """Set which errors should be masked.

    The safety bitmask controls which error conditions are masked (disabled).
    When an error is masked, it will not affect the Status output pin or
    disable TEC operation.

    WARNING: Masking errors may lead to device damage. For example, masking
    INTERNAL_TEMP_HIGH means high internal temperatures will not be reflected
    on the status pin and TEC output will not be disabled.

    Only the following errors can be masked (bits 0-7):
    - ENABLE_PIN_NOT_SET
    - INTERNAL_TEMP_HIGH
    - THERMAL_LATCH_UP
    - CYCLING_TIME_SMALL
    - NO_SENSOR
    - NO_TEC
    - TEC_MISPOLED

    This replaces the entire bitmask with the specified errors.
    Any errors not in the list will be unmasked.

    Args:
        conn: Serial connection object
        *errors: Variable number of ErrorFlags to mask. Only flags
                corresponding to bits 0-7 are valid.

    Raises:
        ValueError: If any error flag is outside the maskable range
            (bits 0-7)

    Example:
        >>> # Mask only internal temperature errors
        >>> safety.set_masked_errors(ErrorFlags.INTERNAL_TEMP_HIGH)
        >>> # Mask multiple errors
        >>> safety.set_masked_errors(
        ...     ErrorFlags.NO_SENSOR, ErrorFlags.CYCLING_TIME_SMALL
        ... )
        >>> # Unmask all errors
        >>> safety.set_masked_errors()

    Note:
        Must use "M" command to save to non-volatile memory.
    """
    bitmask = 0
    for error in errors:
        error_value = int(error)
        # Only bits 0-7 can be masked
        if error_value > 0xFF:
            err_name = error.name
            err_bit = error_value.bit_length() - 1
            raise ValueError(
                f'{err_name} (bit {err_bit}) cannot be masked. '
                f'Only errors corresponding to bits 0-7 can be masked.'
            )
        bitmask |= error_value

    write_command(conn, f'S{bitmask}')


def safety_bitmask_get_masked_errors(conn: serial.Serial) -> list[ErrorFlags]:
    """Read which errors are currently masked.
    Args:
        conn: Serial connection object

    Returns:
        List of ErrorFlags that are currently masked

    Example:
        >>> masked = safety.get_masked_errors()
        >>> if ErrorFlags.INTERNAL_TEMP_HIGH in masked:
        ...     print('Temperature errors are masked!')
    """
    response = query(conn, 'S?')
    bitmask = int(response)
    print(f'error mask: {response}')

    masked = []
    # Check each maskable error (bits 0-7)
    maskable_errors = [
        ErrorFlags.ENABLE_PIN_NOT_SET,
        ErrorFlags.INTERNAL_TEMP_HIGH,
        ErrorFlags.THERMAL_LATCH_UP,
        ErrorFlags.CYCLING_TIME_SMALL,
        ErrorFlags.NO_SENSOR,
        ErrorFlags.NO_TEC,
        ErrorFlags.TEC_MISPOLED,
    ]

    for error in maskable_errors:
        if bitmask & int(error):
            masked.append(error)

    return masked


def safety_bitmask_mask_error(conn: serial.Serial, error: ErrorFlags) -> None:
    """Add an error to the mask without affecting other masked errors.

    Args:
        conn: Serial connection object
        error: ErrorFlags value to mask

    Raises:
        ValueError: If error flag is outside the maskable range (bits 0-7)

    Example:
        >>> # Add INTERNAL_TEMP_HIGH to existing masked errors
        >>> safety.mask_error(ErrorFlags.INTERNAL_TEMP_HIGH)

    Note:
        This reads the current bitmask, adds the specified error,
        and writes it back. Only bits 0-7 can be masked.
    """
    error_value = int(error)
    if error_value > 0xFF:
        err_name = error.name
        err_bit = error_value.bit_length() - 1
        raise ValueError(
            f'{err_name} (bit {err_bit}) cannot be masked. '
            f'Only errors corresponding to bits 0-7 can be masked.'
        )

    current_masked = safety_bitmask_get_masked_errors()
    if error not in current_masked:
        current_masked.append(error)
    safety_bitmask_set_masked_errors(*current_masked)


def safety_bitmask_unmask_error(
    conn: serial.Serial,
    error: ErrorFlags,
) -> None:
    """Remove an error from the mask without affecting other masked errors.

    Args:
        conn: Serial connection object
        error: ErrorFlags value to unmask

    Example:
        >>> # Remove INTERNAL_TEMP_HIGH from masked errors
        >>> safety.unmask_error(ErrorFlags.INTERNAL_TEMP_HIGH)

    Note:
        This reads the current bitmask, removes the specified error,
        and writes it back.
    """
    current_masked = safety_bitmask_get_masked_errors()
    if error in current_masked:
        current_masked.remove(error)
    safety_bitmask_set_masked_errors(*current_masked)


def safety_bitmask_unmask_all(conn: serial.Serial) -> None:
    """Unmask all errors (set bitmask to 0).
    Args:
        conn: Serial connection object

    This is the safest configuration as all errors will be reported.

    Example:
        >>> # Reset to safe defaults
        >>> safety.unmask_all()
    """
    safety_bitmask_set_masked_errors(conn, *[])


def tec_control_set_current_limit(conn: serial.Serial, limit_ma: int) -> None:
    """Set TEC current limit.

    Args:
        conn: Serial connection object
        limit_ma: Current limit in mA (200-2000)

    Raises:
        ValueError: If limit is outside valid range
    """
    if not 200 <= limit_ma <= 2000:
        raise ValueError('Current limit must be between 200 and 2000 mA')
    write_command(conn, f'L{limit_ma}')


def tec_control_get_current_limit(conn: serial.Serial) -> int:
    """Read TEC current limit setting.
    Args:
        conn: Serial connection object

    Returns:
        Current limit in mA
    """
    response = query(conn, 'L?')
    return int(response)


TemperatureMode = Literal['heating', 'cooling']


def tec_control_get_actual_current(
    conn: serial.Serial,
) -> Tuple[int, TemperatureMode]:
    """Read actual TEC current.

    Args:
        conn: Serial connection object

    Returns:
        Tuple of (current in mA, mode string).
        Mode is 'heating' if current < 0, else 'cooling'
    """
    response = query(conn, 'A?')
    current = int(response)
    mode = 'heating' if current < 0 else 'cooling'
    return current, mode


def tec_control_get_actual_voltage(conn: serial.Serial) -> int:
    """Read actual TEC voltage.
    Args:
        conn: Serial connection object

    Returns:
        Voltage in mV
    """
    response = query(conn, 'U?')
    return int(response)


def temperature_control_set_temperature(
    conn: serial.Serial, temp_millidegrees: int
) -> None:
    """Set target temperature.

    Args:
        conn: Serial connection object
        temp_millidegrees: Temperature in millidegrees C (5000-45000).
                         Examples: 5000 = 5°C, 25000 = 25°C

    Raises:
        ValueError: If temperature is outside valid range
    """
    if not 5000 <= temp_millidegrees <= 45000:
        raise ValueError('Temperature must be between 5000 and 45000 (5-45°C)')
    write_command(conn, f'T{temp_millidegrees}')


def temperature_control_get_set_temperature(conn: serial.Serial) -> int:
    """Read set temperature.
    Args:
        conn: Serial connection object

    Returns:
        Set temperature in millidegrees C
    """
    response = query(conn, 'T?')
    return int(response)


def temperature_control_get_actual_temperature(conn: serial.Serial) -> int:
    """Read actual temperature.
    Args:
        conn: Serial connection object

    Returns:
        Actual temperature in millidegrees C
    """
    response = query(conn, 'Te?')
    return int(response)


def temperature_control_set_window(
    conn: serial.Serial,
    window_mk: int,
) -> None:
    """Set temperature window.

    The temperature window defines the acceptable deviation from setpoint.

    Args:
        conn: Serial connection object
        window_mk: Temperature window in millikelvin (1-32000)

    Raises:
        ValueError: If window is outside valid range
    """
    if not 1 <= window_mk <= 32000:
        raise ValueError('Window must be between 1 and 32000 mK')
    write_command(conn, f'W{window_mk}')


def temperature_control_get_window(conn: serial.Serial) -> int:
    """Read temperature window.

    Returns:
        Temperature window in millikelvin
    """
    response = query(conn, 'W?')
    return int(response)


def temperature_control_set_delay(conn: serial.Serial, delay_sec: int) -> None:
    """Set delay time before Status activation.

    This sets the delay between reaching the temperature window and
    activating the Status output pin.

    Args:
        conn: Serial connection object
        delay_sec: Delay in seconds (1-32000)

    Raises:
        ValueError: If delay is outside valid range
    """
    if not 1 <= delay_sec <= 32000:
        raise ValueError('Delay must be between 1 and 32000 seconds')
    write_command(conn, f'd{delay_sec}')


def temperature_control_get_delay(conn: serial.Serial) -> int:
    """Read temperature window delay time.
    Args:
        conn: Serial connection object

    Returns:
        Delay time in seconds
    """
    response = query(conn, 'd?')
    return int(response)


def loop_test_set_critical_gain(conn: serial.Serial, gain: int) -> None:
    """Set critical gain for loop test.

    Args:
        conn: Serial connection object
        gain: Critical gain in mA/K (10-100000)

    Raises:
        ValueError: If gain is outside valid range
    """
    if not 10 <= gain <= 100000:
        raise ValueError(
            'Critical gain must be between 10 and 100000 mA/K',
        )
    write_command(conn, f'G{gain}')


def loop_test_get_critical_gain(conn: serial.Serial) -> int:
    """Read critical gain.
    Args:
        conn: Serial connection object

    Returns:
        Critical gain in mA/K
    """
    response = query(conn, 'G?')
    return int(response)


def loop_test_set_critical_period(conn: serial.Serial, period_ms: int) -> None:
    """Set critical period for loop test.

    Args:
        conn: Serial connection object
        period_ms: Critical period in milliseconds (100-100000)

    Raises:
        ValueError: If period is outside valid range
    """
    if not 100 <= period_ms <= 100000:
        raise ValueError('Critical period must be between 100 and 100000 msec')
    write_command(conn, f'O{period_ms}')


def loop_test_get_critical_period(conn: serial.Serial) -> int:
    """Read critical period.

    Args:
        conn: Serial connection object
    Returns:
        Critical period in milliseconds
    """
    response = query(conn, 'O?')
    return int(response)


def pid_settings_set_p_share(conn: serial.Serial, p_value: int) -> None:
    """Set P Share (proportional gain).

    Args:
        conn: Serial connection object
        p_value: P share in mA/K (0-100000)

    Raises:
        ValueError: If P value is outside valid range
    """
    if not 0 <= p_value <= 100000:
        raise ValueError('P share must be between 0 and 100000 mA/K')
    write_command(conn, f'P{p_value}')


def pid_settings_get_p_share(conn: serial.Serial) -> int:
    """Read P share.
    Args:
        conn: Serial connection object

    Returns:
        P share in mA/K
    """
    response = query(conn, 'P?')
    return int(response)


def pid_settings_set_i_share(conn: serial.Serial, i_value: int) -> None:
    """Set I Share (integral gain).

    Args:
        conn: Serial connection object
        i_value: I share in mA/(K*sec) (0-100000)

    Raises:
        ValueError: If I value is outside valid range
    """
    if not 0 <= i_value <= 100000:
        raise ValueError('I share must be between 0 and 100000 mA/(K*sec)')
    write_command(conn, f'I{i_value}')


def pid_settings_get_i_share(conn: serial.Serial) -> int:
    """Read I share.
    Args:
        conn: Serial connection object

    Returns:
        I share in mA/(K*sec)
    """
    response = query(conn, 'I?')
    return int(response)


def pid_settings_set_d_share(conn: serial.Serial, d_value: int) -> None:
    """Set D Share (derivative gain).

    Args:
        conn: Serial connection object
        d_value: D share in (mA*sec)/K (0-100000)

    Raises:
        ValueError: If D value is outside valid range
    """
    if not 0 <= d_value <= 100000:
        raise ValueError('D share must be between 0 and 100000 (mA*sec)/K')
    write_command(conn, f'D{d_value}')


def pid_settings_get_d_share(conn: serial.Serial) -> int:
    """Read D share.
    Args:
        conn: Serial connection object

    Returns:
        D share in (mA*sec)/K
    """
    response = query(conn, 'D?')
    return int(response)


def pid_settings_set_cycle_time(conn: serial.Serial, cycle_ms: int) -> None:
    """Set PID cycling time.

    Args:
        conn: Serial connection object
        cycle_ms: Cycle time in milliseconds (1-1000)

    Raises:
        ValueError: If cycle time is outside valid range
    """
    if not 1 <= cycle_ms <= 1000:
        raise ValueError('Cycle time must be between 1 and 1000 msec')
    write_command(conn, f'C{cycle_ms}')


def pid_settings_get_cycle_time(conn: serial.Serial) -> int:
    """Read cycling time.
    Args:
        conn: Serial connection object

    Returns:
        Cycle time in milliseconds
    """
    response = query(conn, 'C?')
    return int(response)


def save_configuration_to_memory(conn: serial.Serial) -> None:
    """Save current parameters to non-volatile memory.
    Args:
        conn: Serial connection object

    Saves T, W, L, d, G, O, P, I, D, C, and S parameters.

    Note:
        The MTD1020T has limited erase/write cycles. Changes are not
        stored automatically to protect flash memory longevity.
    """
    write_command(conn, 'M')


class MTD1020T:
    """High-level interface for the MTD1020T TEC (Thermoelectric Cooler)
        Controller.

    This class provides a high level wrapper for the MTD1020T serial command
    interface, allowing users to interact with the device using readable
    properties and methods rather than low-level serial commands.

    The class supports querying system information, managing safety bitmasks,
    controlling TEC current, setting temperature targets, configuring PID
    parameters, and saving configurations to non-volatile memory.

    Example:
        >>> controller = MTD1020T(port='/dev/ttyUSB0')
        >>> print(controller.version)
        >>> controller.current_limit = 1500
        >>> controller.temperature_set_point = 25000  # 25°C
        >>> print(controller.actual_temperature)
        >>> controller.save_to_memory()

    Args:
        port (str): Serial port device (e.g., "/dev/ttyUSB0" or "COM3").
        baudrate (int, optional): Serial communication speed. Defaults to 9600.
        timeout (float, optional): Read timeout in seconds. Defaults to 1.

    Attributes:
        conn (serial.Serial): Active serial connection to the MTD1020T.
        _version (Optional[str]): Cached firmware version string.
        _uuid (Optional[str]): Cached UUID string.
        _masked_errors (Optional[List[ErrorFlags]]): Cached list of masked
            errors.
    """

    def __init__(
        self,
        port,
        baudrate: int = 115200,
        timeout: float = 2.0,
        write_timeout: float = 0.5,
    ):
        """Initialize the MTD1020T serial interface."""

        self.conn = None
        self._version: str | None = None
        self._uuid: str | None = None
        self._masked_errors: Optional[List[ErrorFlags]] = None

        self._connect(
            port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=write_timeout,
        )

    def _connect(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: float = 2.0,
        write_timeout: float = 1.0,
    ) -> None:
        conn = serial.Serial(
            port=port,
            baudrate=baudrate,
            # bytesize=serial.EIGHTBITS,
            # parity=serial.PARITY_NONE,
            # stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=write_timeout,
        )

        # Purge buffers and reset
        conn.flush()
        time.sleep(0.05)
        conn.reset_input_buffer()
        conn.reset_output_buffer()

        self.conn = conn

    def _close(self):
        self.conn.close()

    def __repr__(self) -> str:
        """Return a detailed string representation of the MTD1020T instance.

        Provides a comprehensive summary of the controller state, including
        connection settings, firmware information, TEC parameters, temperature
        control values, safety masks, and PID tuning parameters.

        Returns:
            str: Formatted multi-line summary of the current controller state.
        """
        try:
            version = self._version or sys_info_get_version(self.conn)
        except Exception:
            version = '<unavailable>'

        try:
            uuid = self._uuid or sys_info_get_uuid(self.conn)
        except Exception:
            uuid = '<unavailable>'

        try:
            current_limit = tec_control_get_current_limit(self.conn)
            actual_current, mode = tec_control_get_actual_current(self.conn)
            actual_voltage = tec_control_get_actual_voltage(self.conn)
        except Exception:
            current_limit = actual_current = actual_voltage = '<unavailable>'
            mode = '<unknown>'

        try:
            temp_set = temperature_control_get_set_temperature(self.conn)
            temp_actual = temperature_control_get_actual_temperature(self.conn)
            temp_window = temperature_control_get_window(self.conn)
            temp_delay = temperature_control_get_delay(self.conn)
        except Exception:
            temp_set = temp_actual = temp_window = temp_delay = '<unavailable>'

        try:
            masked = (
                [e.name for e in self._masked_errors]
                if self._masked_errors
                else [
                    e.name for e in safety_bitmask_get_masked_errors(self.conn)
                ]
            )
        except Exception:
            masked = ['<unavailable>']

        try:
            pid_p = pid_settings_get_p_share(self.conn)
            pid_i = pid_settings_get_i_share(self.conn)
            pid_d = pid_settings_get_d_share(self.conn)
            pid_cycle = pid_settings_get_cycle_time(self.conn)
        except Exception:
            pid_p = pid_i = pid_d = pid_cycle = '<unavailable>'

        return (
            f'<MTD1020T Controller>\n'
            f'  Connection\n'
            f'    Port: {self.conn.port}\n'
            f'    Baudrate: {self.conn.baudrate}\n'
            f'    Timeout: {self.conn.timeout}\n\n'
            f'  System Info\n'
            f'    Firmware Version: {version}\n'
            f'    UUID: {uuid}\n\n'
            f'  TEC Control\n'
            f'    Current Limit: {current_limit} mA\n'
            f'    Actual Current: {actual_current} mA ({mode})\n'
            f'    Actual Voltage: {actual_voltage} mV\n\n'
            f'  Temperature Control\n'
            f'    Setpoint: {temp_set} m°C\n'
            f'    Actual: {temp_actual} m°C\n'
            f'    Window: {temp_window} mK\n'
            f'    Delay: {temp_delay} s\n\n'
            f'  PID Parameters\n'
            f'    P Share: {pid_p} mA/K\n'
            f'    I Share: {pid_i} mA/(K·s)\n'
            f'    D Share: {pid_d} (mA·s)/K\n'
            f'    Cycle Time: {pid_cycle} ms\n\n'
            f'  Safety\n'
            f'    Masked Errors: {masked}\n'
        )

    def __enter__(self) -> 'MTD1020T':
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self._close()

    def __del__(self):
        self._close()

    @property
    def version(self) -> str:
        """Get device firmware version.

        Returns:
            str: Firmware version string, e.g., "MTD1020 FW0.6.8".
        """
        if self._version:
            return self._version
        self._version = sys_info_get_version(self.conn)
        return self._version

    @property
    def uuid(self) -> str:
        """Get the device UUID (Universal Unique Identifier).

        Returns:
            str: 32-character hexadecimal UUID string.
        """
        if self._uuid:
            return self._uuid
        self._uuid = sys_info_get_uuid(self.conn)
        return self._uuid

    @property
    def errors(self) -> Dict[str, bool]:
        """Get the currently active error states.

        Returns:
            Dict[str, bool]: Mapping of error flag names to boolean states.
        """
        return sys_info_get_active_errors(self.conn)

    def reset_errors(self) -> None:
        """Reset the device error register."""
        sys_info_reset_error_register(self.conn)

    @property
    def masked_errors(self) -> List[ErrorFlags]:
        """Get currently masked errors.

        Returns:
            List[ErrorFlags]: List of masked error flags.
        """
        if self._masked_errors is None:
            self._masked_errors = safety_bitmask_get_masked_errors(self.conn)
        return self._masked_errors

    @masked_errors.setter
    def masked_errors(self, *errors: ErrorFlags) -> None:
        """Set masked errors (overwrites all existing masks).

        Args:
            *errors (ErrorFlags): Error flags to mask.
        """
        safety_bitmask_set_masked_errors(self.conn, *errors)

    def mask_error(self, error: ErrorFlags) -> None:
        """Add an error to the current mask without overwriting others.

        Args:
            error (ErrorFlags): Error flag to mask.
        """
        safety_bitmask_mask_error(self.conn, error)
        self._masked_errors = safety_bitmask_get_masked_errors(self.conn)

    def unmask_error(self, error: ErrorFlags) -> None:
        """Remove an error from the current mask.

        Args:
            error (ErrorFlags): Error flag to unmask.
        """
        safety_bitmask_unmask_error(self.conn, error)
        self._masked_errors = safety_bitmask_get_masked_errors(self.conn)

    def unmask_all_errors(self) -> None:
        """Unmask all errors (restore safe default state)."""
        safety_bitmask_unmask_all(self.conn)
        self._masked_errors = []

    @property
    def current_limit(self) -> int:
        """Get the TEC current limit.

        Returns:
            int: Current limit in milliamperes (mA).
        """
        return tec_control_get_current_limit(self.conn)

    @current_limit.setter
    def current_limit(self, limit_ma: int) -> None:
        """Set the TEC current limit.

        Args:
            limit_ma (int): Current limit in mA (200–2000).
        """
        tec_control_set_current_limit(self.conn, limit_ma)

    @property
    def actual_current(self) -> Tuple[int, TemperatureMode]:
        """Get actual TEC current and mode.

        Returns:
            Tuple[int, TemperatureMode]: (current_mA, mode)
            where mode is "heating" or "cooling".
        """
        return tec_control_get_actual_current(self.conn)

    @property
    def actual_voltage(self) -> int:
        """Get the actual TEC voltage.

        Returns:
            int: Voltage in millivolts (mV).
        """
        return tec_control_get_actual_voltage(self.conn)

    @property
    def temperature_set_point(self) -> int:
        """Get the current temperature set point.

        Returns:
            int: Temperature setpoint in millidegrees Celsius.
        """
        return temperature_control_get_set_temperature(self.conn)

    @temperature_set_point.setter
    def temperature_set_point(self, temp_millidegrees: int) -> None:
        """Set target temperature.

        Args:
            temp_millidegrees (int): Target temperature in millidegrees Celsius
                (5000–45000).
        """
        temperature_control_set_temperature(self.conn, temp_millidegrees)

    @property
    def actual_temperature(self) -> int:
        """Get the actual temperature reading.

        Returns:
            int: Actual temperature in millidegrees Celsius.
        """
        return temperature_control_get_actual_temperature(self.conn)

    @property
    def temperature(self) -> Tuple[int, int]:
        """Get both the set and actual temperatures.

        Returns:
            Tuple[int, int]: (set_temperature, actual_temperature) in
                millidegrees Celsius.
        """
        return (
            self.temperature_set_point,
            self.actual_temperature,
        )

    @property
    def temperature_set_point_window(self) -> int:
        """Get the configured temperature window.

        Returns:
            int: Temperature window in millikelvin.
        """
        return temperature_control_get_window(self.conn)

    @temperature_set_point_window.setter
    def temperature_set_point_window(self, window_mk: int) -> None:
        """Set the temperature window.

        Args:
            window_mk (int): Window width in millikelvin (1–32000).
        """
        temperature_control_set_window(self.conn, window_mk)

    @property
    def temperature_control_delay(self) -> int:
        """Get the temperature control delay time.

        Returns:
            int: Delay time in seconds.
        """
        return temperature_control_get_delay(self.conn)

    @temperature_control_delay.setter
    def temperature_control_delay(self, delay_sec: int) -> None:
        """Set the delay before status activation.

        Args:
            delay_sec (int): Delay time in seconds (1–32000).
        """
        temperature_control_set_delay(self.conn, delay_sec)

    @property
    def loop_test_critical_gain(self) -> int:
        """Get the critical gain value for loop testing.

        Returns:
            int: Critical gain in mA/K.
        """
        return loop_test_get_critical_gain(self.conn)

    @loop_test_critical_gain.setter
    def loop_test_critical_gain(self, gain: int) -> None:
        """Set the critical gain value for loop testing.

        Args:
            gain (int): Critical gain in mA/K (10–100000).
        """
        loop_test_set_critical_gain(self.conn, gain)

    @property
    def loop_test_critical_period(self) -> int:
        """Get the critical period for loop testing.

        Returns:
            int: Period in milliseconds.
        """
        return loop_test_get_critical_period(self.conn)

    @loop_test_critical_period.setter
    def loop_test_critical_period(self, period_ms: int) -> None:
        """Set the critical period for loop testing.

        Args:
            period_ms (int): Period in milliseconds (100–100000).
        """
        loop_test_set_critical_period(self.conn, period_ms)

    @property
    def pid_p_share(self) -> int:
        """Get the PID proportional gain (P share).

        Returns:
            int: P share in mA/K.
        """
        return pid_settings_get_p_share(self.conn)

    @pid_p_share.setter
    def pid_p_share(self, p_value: int) -> None:
        """Set the PID proportional gain (P share).

        Args:
            p_value (int): P share in mA/K (0–100000).
        """
        pid_settings_set_p_share(self.conn, p_value)

    @property
    def pid_i_share(self) -> int:
        """Get the PID integral gain (I share).

        Returns:
            int: I share in mA/(K·s).
        """
        return pid_settings_get_i_share(self.conn)

    @pid_i_share.setter
    def pid_i_share(self, i_value: int) -> None:
        """Set the PID integral gain (I share).

        Args:
            i_value (int): I share in mA/(K·s) (0–100000).
        """
        pid_settings_set_i_share(self.conn, i_value)

    @property
    def pid_d_share(self) -> int:
        """Get the PID derivative gain (D share).

        Returns:
            int: D share in (mA·s)/K.
        """
        return pid_settings_get_d_share(self.conn)

    @pid_d_share.setter
    def pid_d_share(self, d_value: int) -> None:
        """Set the PID derivative gain (D share).

        Args:
            d_value (int): D share in (mA·s)/K (0–100000).
        """
        pid_settings_set_d_share(self.conn, d_value)

    @property
    def pid_cycle_time(self) -> int:
        """Get the PID cycle time.

        Returns:
            int: Cycle time in milliseconds.
        """
        return pid_settings_get_cycle_time(self.conn)

    @pid_cycle_time.setter
    def pid_cycle_time(self, cycle_time_value: int) -> None:
        """Set the PID cycle time.

        Args:
            cycle_time_value (int): Cycle time in milliseconds (1–1000).
        """
        pid_settings_set_cycle_time(self.conn, cycle_time_value)

    def save_to_memory(self) -> None:
        """Save all current configuration parameters to non-volatile memory."""
        save_configuration_to_memory(self.conn)
