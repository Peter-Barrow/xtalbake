"""
MTD1020T TEC Controller Serial Interface

Implements serial communication and protocol interfaces for the MTD1020T device.
"""

import serial
import time
from typing import Tuple, Optional, List, Literal
from enum import IntFlag

from ._tec_protocol import Severity, ErrorFlags

__all__ = [
    'MTD1020T',
    'write_command',
    'read_response',
    'query',
]


# ERROR FLAGS DEFINITION

MTD_ERRORS = ErrorFlags.from_definitions(
    name='MTD1020ErrorFlags',
    definitions={
        1: (
            'ENABLE_PIN_NOT_SET',
            'Enable pin not set to L (GND)',
            Severity.WARNING,
        ),
        2: (
            'INTERNAL_TEMP_HIGH',
            'Internal temperature too high',
            Severity.CRITICAL,
        ),
        4: (
            'THERMAL_LATCH_UP',
            'TEC current at limit without temperature improvement',
            Severity.CRITICAL,
        ),
        8: (
            'CYCLING_TIME_SMALL',
            'Cycling time too small',
            Severity.WARNING,
        ),
        16: (
            'NO_SENSOR',
            'No sensor detected',
            Severity.CRITICAL,
        ),
        32: (
            'NO_TEC',
            'No TEC detected (connection open)',
            Severity.CRITICAL,
        ),
        64: (
            'TEC_MISPOLED',
            'TEC mispoled (reversed polarity)',
            Severity.CRITICAL,
        ),
        8192: (
            'VALUE_OUT_OF_RANGE',
            'Value out of range',
            Severity.WARNING,
        ),
        16384: (
            'INVALID_COMMAND',
            'Invalid command',
            Severity.WARNING,
        ),
    },
)


# For backward compatibility with existing code
class LegacyErrorFlags(IntFlag):
    """Legacy ErrorFlags enum for backward compatibility.

    Use MTD_ERRORS for new code.
    """

    ENABLE_PIN_NOT_SET = 1 << 0
    INTERNAL_TEMP_HIGH = 1 << 1
    THERMAL_LATCH_UP = 1 << 2
    CYCLING_TIME_SMALL = 1 << 3
    NO_SENSOR = 1 << 4
    NO_TEC = 1 << 5
    TEC_MISPOLED = 1 << 6
    VALUE_OUT_OF_RANGE = 1 << 13
    INVALID_COMMAND = 1 << 14


# Alias for backward compatibility
ErrorFlags = LegacyErrorFlags


# Serial communication


def write_command(conn: serial.Serial, command: str) -> None:
    """Write a command to the MTD1020T device.

    Args:
        conn: Serial connection object.
        command: Command string (e.g., "L?" or "Lx1500").
    """
    cmd_bytes = f'{command}\n'.encode('ascii')
    _ = conn.write(cmd_bytes)
    conn.flush()


def read_response(conn: serial.Serial, timeout: float = 2.0) -> str:
    """Read response from MTD1020T device until line feed terminator.

    Args:
        conn: Serial connection object.
        timeout: Read timeout in seconds.

    Returns:
        Response string with brackets and whitespace stripped.
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
        conn: Serial connection object.
        command: Command string.
        timeout: Read timeout in seconds.

    Returns:
        Response string.
    """
    write_command(conn, command)
    response = read_response(conn, timeout)
    if response == b'':
        time.sleep(0.1)
        write_command(conn, command)
        response = read_response(conn, timeout)
    return response


# SYSTEM INFORMATION COMMANDS


def sys_info_get_version(conn: serial.Serial) -> str:
    """Read hardware and software version.

    Args:
        conn: Serial connection object.

    Returns:
        Version string (e.g., "MTD1020 FW0.6.8").
    """
    return query(conn, 'm?')


def sys_info_get_uuid(conn: serial.Serial) -> str:
    """Read UUID (Universal Unique Identifier).

    Args:
        conn: Serial connection object.

    Returns:
        32-character hexadecimal UUID string.
    """
    return query(conn, 'u?')


def sys_info_get_error_register(conn: serial.Serial) -> int:
    """Read Error Register as raw integer value.

    Args:
        conn: Serial connection object.

    Returns:
        16-bit error register value.
    """
    response = query(conn, 'E?')
    return int(response)


def sys_info_reset_error_register(conn: serial.Serial) -> None:
    """Reset the Error Register.

    Args:
        conn: Serial connection object.

    Note:
        Can also be reset by toggling the Enable pin Off and On again.
    """
    write_command(conn, 'c')


# SAFETY BITMASK COMMANDS


def safety_bitmask_set_masked_errors(
    conn: serial.Serial, *errors: LegacyErrorFlags
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
        conn: Serial connection object.
        *errors: Variable number of ErrorFlags to mask. Only flags
                corresponding to bits 0-7 are valid.

    Raises:
        ValueError: If any error flag is outside the maskable range (bits 0-7).

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


def safety_bitmask_get_masked_errors(
    conn: serial.Serial,
) -> List[LegacyErrorFlags]:
    """Read which errors are currently masked.

    Args:
        conn: Serial connection object.

    Returns:
        List of ErrorFlags that are currently masked.
    """
    response = query(conn, 'S?')
    bitmask = int(response)

    masked = []
    # Check each maskable error (bits 0-7)
    maskable_errors = [
        LegacyErrorFlags.ENABLE_PIN_NOT_SET,
        LegacyErrorFlags.INTERNAL_TEMP_HIGH,
        LegacyErrorFlags.THERMAL_LATCH_UP,
        LegacyErrorFlags.CYCLING_TIME_SMALL,
        LegacyErrorFlags.NO_SENSOR,
        LegacyErrorFlags.NO_TEC,
        LegacyErrorFlags.TEC_MISPOLED,
    ]

    for error in maskable_errors:
        if bitmask & int(error):
            masked.append(error)

    return masked


def safety_bitmask_mask_error(
    conn: serial.Serial, error: LegacyErrorFlags
) -> None:
    """Add an error to the mask without affecting other masked errors.

    Args:
        conn: Serial connection object.
        error: ErrorFlags value to mask.

    Raises:
        ValueError: If error flag is outside the maskable range (bits 0-7).

    Note:
        This reads the current bitmask, adds the specified error, and writes it back.
    """
    error_value = int(error)
    if error_value > 0xFF:
        err_name = error.name
        err_bit = error_value.bit_length() - 1
        raise ValueError(
            f'{err_name} (bit {err_bit}) cannot be masked. '
            f'Only errors corresponding to bits 0-7 can be masked.'
        )

    current_masked = safety_bitmask_get_masked_errors(conn)
    if error not in current_masked:
        current_masked.append(error)
    safety_bitmask_set_masked_errors(conn, *current_masked)


def safety_bitmask_unmask_error(
    conn: serial.Serial, error: LegacyErrorFlags
) -> None:
    """Remove an error from the mask without affecting other masked errors.

    Args:
        conn: Serial connection object.
        error: ErrorFlags value to unmask.

    Note:
        This reads the current bitmask, removes the specified error, and writes it back.
    """
    current_masked = safety_bitmask_get_masked_errors(conn)
    if error in current_masked:
        current_masked.remove(error)
    safety_bitmask_set_masked_errors(conn, *current_masked)


def safety_bitmask_unmask_all(conn: serial.Serial) -> None:
    """Unmask all errors (set bitmask to 0).

    Args:
        conn: Serial connection object.

    This is the safest configuration as all errors will be reported.
    """
    safety_bitmask_set_masked_errors(conn)


# TEC CONTROL COMMANDS


def tec_control_set_current_limit(conn: serial.Serial, limit_ma: int) -> None:
    """Set TEC current limit.

    Args:
        conn: Serial connection object.
        limit_ma: Current limit in mA (200-2000).

    Raises:
        ValueError: If limit is outside valid range.
    """
    if not 200 <= limit_ma <= 2000:
        raise ValueError('Current limit must be between 200 and 2000 mA')
    write_command(conn, f'L{limit_ma}')


def tec_control_get_current_limit(conn: serial.Serial) -> int:
    """Read TEC current limit setting.

    Args:
        conn: Serial connection object.

    Returns:
        Current limit in mA.
    """
    response = query(conn, 'L?')
    return int(response)


TemperatureMode = Literal['heating', 'cooling']


def tec_control_get_actual_current(
    conn: serial.Serial,
) -> Tuple[int, TemperatureMode]:
    """Read actual TEC current.

    Args:
        conn: Serial connection object.

    Returns:
        Tuple of (current in mA, mode string).
        Mode is 'heating' if current < 0, else 'cooling'.
    """
    response = query(conn, 'A?')
    current = int(response)
    mode = 'heating' if current < 0 else 'cooling'
    return current, mode


def tec_control_get_actual_voltage(conn: serial.Serial) -> int:
    """Read actual TEC voltage.

    Args:
        conn: Serial connection object.

    Returns:
        Voltage in mV.
    """
    response = query(conn, 'U?')
    return int(response)


# TEMPERATURE CONTROL COMMANDS


def temperature_control_set_temperature(
    conn: serial.Serial, temp_millidegrees: int
) -> None:
    """Set target temperature.

    Args:
        conn: Serial connection object.
        temp_millidegrees: Temperature in millidegrees C (5000-45000).
                         Examples: 5000 = 5°C, 25000 = 25°C.

    Raises:
        ValueError: If temperature is outside valid range.
    """
    if not 5000 <= temp_millidegrees <= 45000:
        raise ValueError('Temperature must be between 5000 and 45000 (5-45°C)')
    write_command(conn, f'T{temp_millidegrees}')


def temperature_control_get_set_temperature(conn: serial.Serial) -> int:
    """Read set temperature.

    Args:
        conn: Serial connection object.

    Returns:
        Set temperature in millidegrees C.
    """
    response = query(conn, 'T?')
    return int(response)


def temperature_control_get_actual_temperature(conn: serial.Serial) -> int:
    """Read actual temperature.

    Args:
        conn: Serial connection object.

    Returns:
        Actual temperature in millidegrees C.
    """
    response = query(conn, 'Te?')
    return int(response)


def temperature_control_set_window(conn: serial.Serial, window_mk: int) -> None:
    """Set temperature window.

    The temperature window defines the acceptable deviation from setpoint.

    Args:
        conn: Serial connection object.
        window_mk: Temperature window in millikelvin (1-32000).

    Raises:
        ValueError: If window is outside valid range.
    """
    if not 1 <= window_mk <= 32000:
        raise ValueError('Window must be between 1 and 32000 mK')
    write_command(conn, f'W{window_mk}')


def temperature_control_get_window(conn: serial.Serial) -> int:
    """Read temperature window.

    Args:
        conn: Serial connection object.

    Returns:
        Temperature window in millikelvin.
    """
    response = query(conn, 'W?')
    return int(response)


def temperature_control_set_delay(conn: serial.Serial, delay_sec: int) -> None:
    """Set delay time before Status activation.

    This sets the delay between reaching the temperature window and
    activating the Status output pin.

    Args:
        conn: Serial connection object.
        delay_sec: Delay in seconds (1-32000).

    Raises:
        ValueError: If delay is outside valid range.
    """
    if not 1 <= delay_sec <= 32000:
        raise ValueError('Delay must be between 1 and 32000 seconds')
    write_command(conn, f'd{delay_sec}')


def temperature_control_get_delay(conn: serial.Serial) -> int:
    """Read temperature window delay time.

    Args:
        conn: Serial connection object.

    Returns:
        Delay time in seconds.
    """
    response = query(conn, 'd?')
    return int(response)


# LOOP TEST COMMANDS


def loop_test_set_critical_gain(conn: serial.Serial, gain: int) -> None:
    """Set critical gain for loop test.

    Args:
        conn: Serial connection object.
        gain: Critical gain in mA/K (10-100000).

    Raises:
        ValueError: If gain is outside valid range.
    """
    if not 10 <= gain <= 100000:
        raise ValueError('Critical gain must be between 10 and 100000 mA/K')
    write_command(conn, f'G{gain}')


def loop_test_get_critical_gain(conn: serial.Serial) -> int:
    """Read critical gain.

    Args:
        conn: Serial connection object.

    Returns:
        Critical gain in mA/K.
    """
    response = query(conn, 'G?')
    return int(response)


def loop_test_set_critical_period(conn: serial.Serial, period_ms: int) -> None:
    """Set critical period for loop test.

    Args:
        conn: Serial connection object.
        period_ms: Critical period in milliseconds (100-100000).

    Raises:
        ValueError: If period is outside valid range.
    """
    if not 100 <= period_ms <= 100000:
        raise ValueError('Critical period must be between 100 and 100000 msec')
    write_command(conn, f'O{period_ms}')


def loop_test_get_critical_period(conn: serial.Serial) -> int:
    """Read critical period.

    Args:
        conn: Serial connection object.

    Returns:
        Critical period in milliseconds.
    """
    response = query(conn, 'O?')
    return int(response)


# PID SETTINGS COMMANDS


def pid_settings_set_p_share(conn: serial.Serial, p_value: int) -> None:
    """Set P Share (proportional gain).

    Args:
        conn: Serial connection object.
        p_value: P share in mA/K (0-100000).

    Raises:
        ValueError: If P value is outside valid range.
    """
    if not 0 <= p_value <= 100000:
        raise ValueError('P share must be between 0 and 100000 mA/K')
    write_command(conn, f'P{p_value}')


def pid_settings_get_p_share(conn: serial.Serial) -> int:
    """Read P share.

    Args:
        conn: Serial connection object.

    Returns:
        P share in mA/K.
    """
    response = query(conn, 'P?')
    return int(response)


def pid_settings_set_i_share(conn: serial.Serial, i_value: int) -> None:
    """Set I Share (integral gain).

    Args:
        conn: Serial connection object.
        i_value: I share in mA/(K*sec) (0-100000).

    Raises:
        ValueError: If I value is outside valid range.
    """
    if not 0 <= i_value <= 100000:
        raise ValueError('I share must be between 0 and 100000 mA/(K*sec)')
    write_command(conn, f'I{i_value}')


def pid_settings_get_i_share(conn: serial.Serial) -> int:
    """Read I share.

    Args:
        conn: Serial connection object.

    Returns:
        I share in mA/(K*sec).
    """
    response = query(conn, 'I?')
    return int(response)


def pid_settings_set_d_share(conn: serial.Serial, d_value: int) -> None:
    """Set D Share (derivative gain).

    Args:
        conn: Serial connection object.
        d_value: D share in (mA*sec)/K (0-100000).

    Raises:
        ValueError: If D value is outside valid range.
    """
    if not 0 <= d_value <= 100000:
        raise ValueError('D share must be between 0 and 100000 (mA*sec)/K')
    write_command(conn, f'D{d_value}')


def pid_settings_get_d_share(conn: serial.Serial) -> int:
    """Read D share.

    Args:
        conn: Serial connection object.

    Returns:
        D share in (mA*sec)/K.
    """
    response = query(conn, 'D?')
    return int(response)


def pid_settings_set_cycle_time(conn: serial.Serial, cycle_ms: int) -> None:
    """Set PID cycling time.

    Args:
        conn: Serial connection object.
        cycle_ms: Cycle time in milliseconds (1-1000).

    Raises:
        ValueError: If cycle time is outside valid range.
    """
    if not 1 <= cycle_ms <= 1000:
        raise ValueError('Cycle time must be between 1 and 1000 msec')
    write_command(conn, f'C{cycle_ms}')


def pid_settings_get_cycle_time(conn: serial.Serial) -> int:
    """Read cycling time.

    Args:
        conn: Serial connection object.

    Returns:
        Cycle time in milliseconds.
    """
    response = query(conn, 'C?')
    return int(response)


# CONFIGURATION SAVE COMMAND


def save_configuration_to_memory(conn: serial.Serial) -> None:
    """Save current parameters to non-volatile memory.

    Args:
        conn: Serial connection object.

    Saves T, W, L, d, G, O, P, I, D, C, and S parameters.

    Note:
        The MTD1020T has limited erase/write cycles. Changes are not
        stored automatically to protect flash memory longevity.
    """
    write_command(conn, 'M')


# MTD1020T CONTROLLER CLASS


class MTD1020T:
    """MTD1020T TEC (Thermoelectric Cooler) Controller.

    High-level interface implementing temperature controller protocols.
    Supports context manager for automatic cleanup.

    Example:
        >>> with MTD1020T(port='/dev/ttyUSB0') as tec:
        ...     tec.set_temperature_setpoint(25.0)
        ...     print(tec.get_actual_temperature())
        ...     tec.save_configuration()

    Implements protocols:
        - CoreTemperatureControl
        - DeviceIdentification
        - ErrorReporting
        - PIDConfiguration
        - PowerMonitoring
        - ConfigurationPersistence
        - ErrorMasking
        - CurrentLimitControl
        - ActualVoltageMonitoring
        - TemperatureStabilityMonitoring
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 2.0,
        write_timeout: float = 0.5,
    ):
        """Initialize MTD1020T controller.

        Args:
            port: Serial port (e.g., '/dev/ttyUSB0', 'COM3').
            baudrate: Serial baud rate (default: 115200).
            timeout: Read timeout in seconds.
            write_timeout: Write timeout in seconds.
        """
        self.conn: Optional[serial.Serial] = None
        self.errors = MTD_ERRORS

        self._connect(
            port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=write_timeout,
        )

    def _connect(
        self,
        port: str,
        baudrate: int,
        timeout: float,
        write_timeout: float,
    ) -> None:
        """Establish serial connection to device.

        Args:
            port: Serial port.
            baudrate: Baud rate.
            timeout: Read timeout.
            write_timeout: Write timeout.
        """
        conn = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=write_timeout,
        )

        # Purge buffers and reset
        conn.flush()
        time.sleep(0.05)
        conn.reset_input_buffer()
        conn.reset_output_buffer()

        self.conn = conn

    def _close(self) -> None:
        """Close serial connection."""
        if self.conn is not None:
            self.conn.close()

    def __enter__(self) -> 'MTD1020T':
        """Context manager entry."""
        return self

    def __exit__(self, exception_type, exception_value, traceback) -> None:
        """Context manager exit with cleanup."""
        self._close()

    def __del__(self) -> None:
        """Cleanup on deletion."""
        self._close()

    # CoreTemperatureControl Protocol

    def get_temperature_setpoint(self) -> float:
        """Get current temperature setpoint.

        Returns:
            Temperature setpoint in degrees Celsius.
        """
        temp_millidegrees = temperature_control_get_set_temperature(self.conn)
        return temp_millidegrees / 1000.0

    def set_temperature_setpoint(self, temp_celsius: float) -> None:
        """Set target temperature.

        Args:
            temp_celsius: Target temperature in degrees Celsius.
        """
        temp_millidegrees = int(temp_celsius * 1000)
        temperature_control_set_temperature(self.conn, temp_millidegrees)

    def get_actual_temperature(self) -> float:
        """Get actual measured temperature.

        Returns:
            Actual temperature in degrees Celsius.
        """
        temp_millidegrees = temperature_control_get_actual_temperature(
            self.conn
        )
        return temp_millidegrees / 1000.0

    # DeviceIdentification Protocol

    def get_firmware_version(self) -> str:
        """Get device firmware version.

        Returns:
            Firmware version string.
        """
        return sys_info_get_version(self.conn)

    def get_device_id(self) -> str:
        """Get unique device identifier.

        Returns:
            Device UUID.
        """
        return sys_info_get_uuid(self.conn)

    # ErrorReporting Protocol

    def get_error_code(self) -> int:
        """Get current error code.

        Returns:
            Integer error code representing active error flags.
        """
        return sys_info_get_error_register(self.conn)

    def has_errors(self) -> bool:
        """Check if any errors are present.

        Returns:
            True if any errors are active.
        """
        return self.get_error_code() != 0

    def clear_errors(self) -> None:
        """Clear/reset all error conditions."""
        sys_info_reset_error_register(self.conn)

    # PIDConfiguration Protocol

    def get_pid_parameters(self) -> Tuple[float, float, float]:
        """Get PID controller parameters.

        Returns:
            Tuple of (P, I, D) parameters in standard units.
        """
        p = pid_settings_get_p_share(self.conn) / 1000.0  # Convert mA/K to A/K
        i = (
            pid_settings_get_i_share(self.conn) / 1000.0
        )  # Convert mA/(K·s) to A/(K·s)
        d = (
            pid_settings_get_d_share(self.conn) / 1000.0
        )  # Convert (mA·s)/K to (A·s)/K
        return (p, i, d)

    def set_pid_parameters(self, p: float, i: float, d: float) -> None:
        """Set PID controller parameters.

        Args:
            p: Proportional gain in A/K.
            i: Integral gain in A/(K·s).
            d: Derivative gain in (A·s)/K.
        """
        p_ma = int(p * 1000)  # Convert A/K to mA/K
        i_ma = int(i * 1000)  # Convert A/(K·s) to mA/(K·s)
        d_ma = int(d * 1000)  # Convert (A·s)/K to (mA·s)/K

        pid_settings_set_p_share(self.conn, p_ma)
        pid_settings_set_i_share(self.conn, i_ma)
        pid_settings_set_d_share(self.conn, d_ma)

    # PowerMonitoring Protocol

    def get_output_power(self) -> float:
        """Get current output power.

        Returns:
            Output current in amperes (absolute value).
        """
        current_ma, _ = tec_control_get_actual_current(self.conn)
        return abs(current_ma) / 1000.0

    def get_supply_voltage(self) -> float:
        """Get supply voltage.

        Returns:
            Actual TEC voltage in volts.
        """
        voltage_mv = tec_control_get_actual_voltage(self.conn)
        return voltage_mv / 1000.0

    # ConfigurationPersistence Protocol

    def save_configuration(self) -> None:
        """Save current configuration to non-volatile memory.

        Persists all settings so they survive power cycles.

        Note:
            The MTD1020T has limited erase/write cycles. Only call this
            when you want to permanently save settings.
        """
        save_configuration_to_memory(self.conn)

    # ErrorMasking Protocol

    def get_masked_errors(self) -> int:
        """Get currently masked error flags.

        Returns:
            Bitmask of masked error flags.
        """
        masked_list = safety_bitmask_get_masked_errors(self.conn)
        mask = 0
        for error in masked_list:
            mask |= int(error)
        return mask

    def mask_error(self, error_bit: int) -> None:
        """Mask (ignore) a specific error.

        Args:
            error_bit: Error bit value to mask (must be in bits 0-7).
        """
        # Convert bit value to legacy ErrorFlags
        if error_bit in [1, 2, 4, 8, 16, 32, 64]:
            error_flag = LegacyErrorFlags(error_bit)
            safety_bitmask_mask_error(self.conn, error_flag)
        else:
            raise ValueError(f'Error bit {error_bit} cannot be masked')

    def unmask_error(self, error_bit: int) -> None:
        """Unmask (re-enable) a specific error.

        Args:
            error_bit: Error bit value to unmask.
        """
        # Convert bit value to legacy ErrorFlags
        if error_bit in [1, 2, 4, 8, 16, 32, 64]:
            error_flag = LegacyErrorFlags(error_bit)
            safety_bitmask_unmask_error(self.conn, error_flag)
        else:
            raise ValueError(f'Error bit {error_bit} cannot be unmasked')

    def unmask_all_errors(self) -> None:
        """Unmask all errors (restore default safe state)."""
        safety_bitmask_unmask_all(self.conn)

    # CurrentLimitControl Protocol

    def get_current_limit(self) -> float:
        """Get TEC current limit.

        Returns:
            Current limit in amperes.
        """
        limit_ma = tec_control_get_current_limit(self.conn)
        return limit_ma / 1000.0

    def set_current_limit(self, limit_amps: float) -> None:
        """Set TEC current limit.

        Args:
            limit_amps: Current limit in amperes (0.2 to 2.0).
        """
        limit_ma = int(limit_amps * 1000)
        tec_control_set_current_limit(self.conn, limit_ma)

    def get_actual_current(self) -> Tuple[float, str]:
        """Get actual TEC current and direction.

        Returns:
            Tuple of (current_amps, mode) where mode is "heating" or "cooling".
        """
        current_ma, mode = tec_control_get_actual_current(self.conn)
        return (abs(current_ma) / 1000.0, mode)

    # ActualVoltageMonitoring Protocol

    def get_actual_voltage(self) -> float:
        """Get actual output voltage.

        Returns:
            Output voltage in volts.
        """
        voltage_mv = tec_control_get_actual_voltage(self.conn)
        return voltage_mv / 1000.0

    # TemperatureStabilityMonitoring Protocol

    def get_temperature_stability_status(self) -> bool:
        """Get temperature stability status.

        Returns:
            True if temperature is within stability window and delay has passed.

        Note:
            MTD1020T doesn't have a direct stability status query.
            This checks if actual temp is within the configured window.
        """
        setpoint = temperature_control_get_set_temperature(self.conn)
        actual = temperature_control_get_actual_temperature(self.conn)
        window = temperature_control_get_window(self.conn)

        # Window is in millikelvin, temperatures in millidegrees
        deviation = abs(actual - setpoint)
        return deviation <= window

    def get_stability_window(self) -> Tuple[float, float]:
        """Get temperature stability window.

        Returns:
            Tuple of (low_threshold, high_threshold) in degrees Celsius.
            Both values are symmetric around setpoint.
        """
        window_mk = temperature_control_get_window(self.conn)
        window_celsius = window_mk / 1000.0
        return (-window_celsius, window_celsius)

    def set_stability_window(self, low: float, high: float) -> None:
        """Set temperature stability window.

        Args:
            low: Low threshold in degrees Celsius (should be negative).
            high: High threshold in degrees Celsius (should be positive).

        Note:
            MTD1020T only supports symmetric windows. This will use the
            larger of |low| and |high| as the window size.
        """
        window_celsius = max(abs(low), abs(high))
        window_mk = int(window_celsius * 1000)
        temperature_control_set_window(self.conn, window_mk)

    # Additional

    def get_error_report(self, colorize: bool = True) -> str:
        """Get formatted error report.

        Args:
            colorize: Whether to include ANSI color codes.

        Returns:
            Formatted error report string.
        """
        error_code = self.get_error_code()
        return self.errors.format_report(error_code, colorize)

    def print_error_report(self, colorize: bool = True) -> None:
        """Print formatted error report.

        Args:
            colorize: Whether to include ANSI color codes.
        """
        print(self.get_error_report(colorize))

    # backward compatibility

    @property
    def version(self) -> str:
        """Get device firmware version (legacy property)."""
        return self.get_firmware_version()

    @property
    def uuid(self) -> str:
        """Get device UUID (legacy property)."""
        return self.get_device_id()

    @property
    def current_limit(self) -> float:
        """Get/set current limit in amperes (legacy property)."""
        return self.get_current_limit()

    @current_limit.setter
    def current_limit(self, limit_amps: float) -> None:
        self.set_current_limit(limit_amps)

    @property
    def temperature_set_point(self) -> float:
        """Get/set temperature setpoint in degrees (legacy property)."""
        return self.get_temperature_setpoint()

    @temperature_set_point.setter
    def temperature_set_point(self, temp_celsius: float) -> None:
        self.set_temperature_setpoint(temp_celsius)

    @property
    def actual_temperature(self) -> float:
        """Get actual temperature in degrees (legacy property)."""
        return self.get_actual_temperature()

    def save_to_memory(self) -> None:
        """Save configuration to memory (legacy method name)."""
        self.save_configuration()
