import serial
import time
from typing import Optional, List, Tuple
from dataclasses import dataclass
from enum import Enum

from ._tec_protocol import Severity, ErrorFlags

OC3_ERRORS = ErrorFlags.from_definitions(
    name='OvenFaultFlags',
    definitions={
        1: (
            'NO_RESPONSE_FROM_ADC',
            'No response from temperature ADC',
            Severity.CRITICAL,
        ),
        2: (
            'OUT_OF_RANGE_ADC_READING',
            'Out of range reading from temperature ADC',
            Severity.CRITICAL,
        ),
        4: (
            'POWER_SUPPLY_LOW_VOLTAGE',
            'Power supply - low voltage',
            Severity.CRITICAL,
        ),
        8: (
            'FAULTY_TEMPERATURE_SENSOR',
            'Measured temperature out of expected values',
            Severity.CRITICAL,
        ),
        16: (
            'HEATING_INHIBITOR_ACTIVE',
            'Heating inhibitor active (external interlock)',
            Severity.WARNING,
        ),
    },
)


# Serial communication


def build_command(
    msg_header: str,
    payload: Optional[List[str]] = None,
) -> str:
    """Build command string from header and payload.

    Args:
        msg_header: Command header (e.g., 'jnn', 'cnn3').
        payload: Optional list of parameter strings.

    Returns:
        Formatted command string.
    """
    if payload:
        return f'{msg_header};' + ';'.join(payload)
    return f'{msg_header};'


def write_command(conn: serial.Serial, command: str) -> None:
    """Write command to serial connection.

    Args:
        conn: Serial connection.
        command: Command string to write.
    """
    cmd_bytes = f'{command}\n'.encode('ascii')
    _ = conn.write(cmd_bytes)
    conn.flush()


def read_response(conn: serial.Serial, timeout: float = 2.0) -> List[str]:
    """Read response from serial connection.

    Args:
        conn: Serial connection.
        timeout: Read timeout in seconds.

    Returns:
        List of response fields split by semicolon.
    """
    original_timeout = conn.timeout
    conn.timeout = timeout

    response = conn.read_until(b'\n')
    conn.flush()

    conn.timeout = original_timeout

    return response.decode('utf-8').strip().split(';')


def query(
    conn: serial.Serial,
    command: str,
    timeout: float = 2.0,
    retry: bool = False,
) -> List[str]:
    """Send command and read response.

    Args:
        conn: Serial connection.
        command: Command string to send.
        timeout: Read timeout in seconds.
        retry: Whether to retry on empty response.

    Returns:
        List of response fields.
    """
    write_command(conn, command)
    response = read_response(conn, timeout)

    if response == b'' and not retry:
        time.sleep(0.1)
        response = query(conn, command, timeout, retry=True)

    return response


@dataclass
class AlarmAndStability:
    """Alarm and stability configuration.

    Command: cnn3
    """

    alarm_low: float
    alarm_high: float
    stability_low: float
    stability_high: float
    set_point_minimum: float
    set_point_maximum: float

    @classmethod
    def read(cls, conn: serial.Serial) -> 'AlarmAndStability':
        """Read alarm and stability settings from device.

        Args:
            conn: Serial connection.

        Returns:
            AlarmAndStability instance.
        """
        command = build_command('cnn3')
        payload = query(conn, command)
        return cls.unpack(payload)

    @classmethod
    def unpack(cls, payload: List[str]) -> 'AlarmAndStability':
        """Unpack payload into AlarmAndStability.

        Args:
            payload: Response payload from device.

        Returns:
            AlarmAndStability instance.
        """
        # Parse response format from cnn3 command
        # Expected format: cnn3;alarm_low;alarm_high;stability_low;stability_high;sp_min;sp_max
        return cls(
            alarm_low=float(payload[1]),
            alarm_high=float(payload[2]),
            stability_low=float(payload[3]),
            stability_high=float(payload[4]),
            set_point_minimum=float(payload[5]),
            set_point_maximum=float(payload[6]),
        )

    def write(self, conn: serial.Serial) -> 'AlarmAndStability':
        """Write alarm and stability settings to device.

        Args:
            conn: Serial connection.

        Returns:
            Updated AlarmAndStability read back from device.
        """
        payload = [
            f'{self.alarm_low:.1f}',
            f'{self.alarm_high:.1f}',
            f'{self.stability_low:.2f}',
            f'{self.stability_high:.2f}',
            f'{self.set_point_minimum:.1f}',
            f'{self.set_point_maximum:.1f}',
        ]
        command = build_command('dnn3', payload)
        result_payload = query(conn, command)
        return self.unpack(result_payload)


class ControlStatus(Enum):
    """Controller status."""

    OFF = 0
    ACTIVE_CONTROL = 1
    TEMPERATURE_CYCLE = 5


class CycleMode(Enum):
    """Temperature cycling mode."""

    NO_CYCLE = 0
    RUNNING_CYCLE = 1


@dataclass
class Status:
    """Comprehensive device status.

    Command: jnn
    """

    temperature_set_point: float
    temperature_actual: float
    control_status: ControlStatus
    output_power: float
    alarm_status: bool
    faults: int
    temperature_okay: bool
    supply_voltage: float
    firmware_version: str
    cycle_count: int
    cycle_mode_status: CycleMode

    @classmethod
    def read(cls, conn: serial.Serial) -> 'Status':
        """Read comprehensive status from device.

        Args:
            conn: Serial connection.

        Returns:
            Status instance.
        """
        command = build_command('jnn')
        payload = query(conn, command)
        return cls.unpack(payload)

    @classmethod
    def unpack(cls, payload: List[str]) -> 'Status':
        """Unpack payload into Status.

        Args:
            payload: Response payload from device.

        Returns:
            Status instance.
        """
        # Parse jnn response format
        # Expected: jnn;setpoint;actual;control_status;output_power;alarm;faults;temp_ok;voltage;version;cycle_count;cycle_mode
        return cls(
            temperature_set_point=float(payload[1]),
            temperature_actual=float(payload[2]),
            control_status=ControlStatus(int(payload[3])),
            output_power=float(payload[4]),
            alarm_status=bool(int(payload[5])),
            faults=int(payload[6]),
            temperature_okay=bool(int(payload[7])),
            supply_voltage=float(payload[8]),
            firmware_version=payload[9],
            cycle_count=int(payload[10]),
            cycle_mode_status=CycleMode(int(payload[11])),
        )


@dataclass
class Setpoint:
    """Temperature setpoint with ramp rate.

    Command: ixx1 (read), gxx1 (write)
    """

    temperature: float
    ramp_rate: float

    @classmethod
    def read(cls, conn: serial.Serial) -> 'Setpoint':
        """Read setpoint from device.

        Args:
            conn: Serial connection.

        Returns:
            Setpoint instance.
        """
        command = build_command('ixx1')
        payload = query(conn, command)
        return cls.unpack(payload)

    @classmethod
    def unpack(cls, payload: List[str]) -> 'Setpoint':
        """Unpack payload into Setpoint.

        Args:
            payload: Response payload from device.

        Returns:
            Setpoint instance.
        """
        # Parse ixx1 response format
        # Expected: ixx1;temperature;ramp_rate
        return cls(
            temperature=float(payload[1]),
            ramp_rate=float(payload[2]),
        )

    def write(self, conn: serial.Serial) -> 'Setpoint':
        """Write setpoint to device.

        Args:
            conn: Serial connection.

        Returns:
            Updated Setpoint read back from device.
        """
        payload = [
            f'{self.temperature:.1f}',
            f'{self.ramp_rate:.2f}',
        ]
        command = build_command('gxx1', payload)
        result_payload = query(conn, command)
        return self.unpack(result_payload)


class PIDControlMode(Enum):
    """PID control mode."""

    NONE = 1
    PROPORTIONAL_ONLY = 2
    PROPORTIONAL_INTEGRAL = 3
    FULL = 4


@dataclass
class PIDValues:
    """PID controller parameters.

    Command: !bxx (read), !axx (write)
    """

    control_mode: PIDControlMode
    term_proportional: float
    term_integral: float
    term_derivative: float
    deadband: float
    output_enabled: bool

    def write(self, conn: serial.Serial) -> 'PIDValues':
        """Write PID values to device.

        Args:
            conn: Serial connection.

        Returns:
            Updated PIDValues read back from device.
        """
        cntrl = f'{self.control_mode.value}'
        term_p = f'{self.term_proportional:.2f}'
        term_i = f'{self.term_integral:.2f}'
        term_d = f'{self.term_derivative:.2f}'
        deadband = f'{self.deadband:.2f}'
        output = f'{int(self.output_enabled)}'

        payload = [
            cntrl,
            term_p,
            term_i,
            term_d,
            deadband,
            output,
        ]

        command = build_command('!axx', payload)
        result_payload = query(conn, command)
        return PIDValues.unpack(result_payload)

    @classmethod
    def read(cls, conn: serial.Serial) -> 'PIDValues':
        """Read PID values from device.

        Args:
            conn: Serial connection.

        Returns:
            PIDValues instance.
        """
        command = build_command('!bxx')
        payload = query(conn, command)
        return cls.unpack(payload)

    @classmethod
    def unpack(cls, payload: List[str]) -> 'PIDValues':
        """Unpack payload into PIDValues.

        Args:
            payload: Response payload from device.

        Returns:
            PIDValues instance.
        """
        header = payload[0]
        assert len(header) == 4
        cntrl = PIDControlMode(int(header[-1]))
        header = header[0:3]
        assert header[0] == 'b'

        term_p = float(payload[1])
        term_i = float(payload[2])
        term_d = float(payload[3])
        derivative_filter = int(payload[4])
        assert derivative_filter == 1
        deadband = float(payload[5])
        output = int(payload[6]) == 1
        _ = payload[7]  # checksum, unused

        return cls(
            control_mode=cntrl,
            term_proportional=term_p,
            term_integral=term_i,
            term_derivative=term_d,
            deadband=deadband,
            output_enabled=output,
        )


class OC3:
    """Covesion OC3 Oven Controller.

    High-level interface implementing temperature controller protocols.
    Supports context manager for automatic cleanup.

    Example:
        >>> with OC3(port='/dev/ttyUSB0') as oven:
        ...     oven.set_temperature_setpoint(25.0)
        ...     print(oven.get_actual_temperature())

    Implements protocols:
        - CoreTemperatureControl
        - DeviceIdentification
        - ErrorReporting
        - PIDConfiguration
        - PowerMonitoring
        - AlarmManagement
        - RampControl
        - TemperatureCycling
        - TemperatureStabilityMonitoring
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 19200,
        timeout: float = 0.2,
        write_timeout: float = 0.5,
    ):
        """Initialize OC3 controller.

        Args:
            port: Serial port (e.g., '/dev/ttyUSB0', 'COM3').
            baudrate: Serial baud rate (default: 19200).
            timeout: Read timeout in seconds.
            write_timeout: Write timeout in seconds.
        """
        self.conn: Optional[serial.Serial] = None
        self.errors = OC3_ERRORS

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
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
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

    def __enter__(self) -> 'OC3':
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
        status = Status.read(self.conn)
        return status.temperature_set_point

    def set_temperature_setpoint(self, temp_celsius: float) -> None:
        """Set target temperature.

        Args:
            temp_celsius: Target temperature in degrees Celsius.
        """
        setpoint = Setpoint.read(self.conn)
        setpoint.temperature = temp_celsius
        setpoint.write(self.conn)

    def get_actual_temperature(self) -> float:
        """Get actual measured temperature.

        Returns:
            Actual temperature in degrees Celsius.
        """
        status = Status.read(self.conn)
        return status.temperature_actual

    # DeviceIdentification Protocol

    def get_firmware_version(self) -> str:
        """Get device firmware version.

        Returns:
            Firmware version string.
        """
        status = Status.read(self.conn)
        return status.firmware_version

    def get_device_id(self) -> str:
        """Get unique device identifier.

        Returns:
            Device identifier (serial port for OC3).
        """
        return self.conn.port

    # ErrorReporting Protocol

    def get_error_code(self) -> int:
        """Get current error code.

        Returns:
            Integer error code representing active error flags.
        """
        status = Status.read(self.conn)
        return status.faults

    def has_errors(self) -> bool:
        """Check if any errors are present.

        Returns:
            True if any errors are active.
        """
        return self.get_error_code() != 0

    def clear_errors(self) -> None:
        """Clear/reset all error conditions.

        Note: OC3 may not support explicit error clearing.
        This is a no-op for compatibility.
        """
        # OC3 errors typically clear automatically when condition resolves
        pass

    # PIDConfiguration Protocol

    def get_pid_parameters(self) -> Tuple[float, float, float]:
        """Get PID controller parameters.

        Returns:
            Tuple of (P, I, D) parameters.
        """
        pid = PIDValues.read(self.conn)
        return (pid.term_proportional, pid.term_integral, pid.term_derivative)

    def set_pid_parameters(self, p: float, i: float, d: float) -> None:
        """Set PID controller parameters.

        Args:
            p: Proportional gain.
            i: Integral gain.
            d: Derivative gain.
        """
        pid = PIDValues.read(self.conn)
        pid.term_proportional = p
        pid.term_integral = i
        pid.term_derivative = d
        pid.write(self.conn)

    # PowerMonitoring Protocol

    def get_output_power(self) -> float:
        """Get current output power.

        Returns:
            Output power as percentage (0-100).
        """
        status = Status.read(self.conn)
        return status.output_power

    def get_supply_voltage(self) -> float:
        """Get supply voltage.

        Returns:
            Supply voltage in volts.
        """
        status = Status.read(self.conn)
        return status.supply_voltage

    # AlarmManagement Protocol

    def get_alarm_low(self) -> float:
        """Get low temperature alarm threshold.

        Returns:
            Low alarm threshold in degrees Celsius.
        """
        alarm = AlarmAndStability.read(self.conn)
        return alarm.alarm_low

    def set_alarm_low(self, temp_celsius: float) -> None:
        """Set low temperature alarm threshold.

        Args:
            temp_celsius: Low alarm threshold in degrees Celsius.
        """
        alarm = AlarmAndStability.read(self.conn)
        alarm.alarm_low = temp_celsius
        alarm.write(self.conn)

    def get_alarm_high(self) -> float:
        """Get high temperature alarm threshold.

        Returns:
            High alarm threshold in degrees Celsius.
        """
        alarm = AlarmAndStability.read(self.conn)
        return alarm.alarm_high

    def set_alarm_high(self, temp_celsius: float) -> None:
        """Set high temperature alarm threshold.

        Args:
            temp_celsius: High alarm threshold in degrees Celsius.
        """
        alarm = AlarmAndStability.read(self.conn)
        alarm.alarm_high = temp_celsius
        alarm.write(self.conn)

    def get_alarm_status(self) -> bool:
        """Get current alarm status.

        Returns:
            True if alarm is active.
        """
        status = Status.read(self.conn)
        return status.alarm_status

    # RampControl Protocol

    def get_ramp_rate(self) -> float:
        """Get temperature ramp rate.

        Returns:
            Ramp rate in degrees Celsius per minute.
        """
        setpoint = Setpoint.read(self.conn)
        return setpoint.ramp_rate

    def set_ramp_rate(self, rate_deg_per_min: float) -> None:
        """Set temperature ramp rate.

        Args:
            rate_deg_per_min: Ramp rate in degrees Celsius per minute.
        """
        setpoint = Setpoint.read(self.conn)
        setpoint.ramp_rate = rate_deg_per_min
        setpoint.write(self.conn)

    # TemperatureCycling Protocol

    def get_cycle_count(self) -> int:
        """Get number of completed temperature cycles.

        Returns:
            Number of completed cycles.
        """
        status = Status.read(self.conn)
        return status.cycle_count

    def is_cycling(self) -> bool:
        """Check if temperature cycling is active.

        Returns:
            True if cycling is active.
        """
        status = Status.read(self.conn)
        return status.cycle_mode_status == CycleMode.RUNNING_CYCLE

    # TemperatureStabilityMonitoring Protocol

    def get_temperature_stability_status(self) -> bool:
        """Get temperature stability status.

        Returns:
            True if temperature is stable within configured window.
        """
        status = Status.read(self.conn)
        return status.temperature_okay

    def get_stability_window(self) -> Tuple[float, float]:
        """Get temperature stability window.

        Returns:
            Tuple of (low_threshold, high_threshold) relative to setpoint.
        """
        alarm = AlarmAndStability.read(self.conn)
        return (alarm.stability_low, alarm.stability_high)

    def set_stability_window(self, low: float, high: float) -> None:
        """Set temperature stability window.

        Args:
            low: Low threshold (negative offset from setpoint).
            high: High threshold (positive offset from setpoint).
        """
        alarm = AlarmAndStability.read(self.conn)
        alarm.stability_low = low
        alarm.stability_high = high
        alarm.write(self.conn)

    # Additional

    def get_full_status(self) -> Status:
        """Get comprehensive device status.

        Returns:
            Status dataclass with all device information.
        """
        return Status.read(self.conn)

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
