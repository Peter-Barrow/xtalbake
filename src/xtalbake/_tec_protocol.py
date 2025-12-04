"""Protocol definitions for temperature controller interfaces.

This module defines protocols (interfaces) that temperature controllers must
    implement.
Protocols use structural subtyping - any class with matching methods satisfies
    the protocol.
"""

from typing import Protocol, Tuple, runtime_checkable
from enum import IntFlag, Enum
from typing import List, Dict
from dataclasses import dataclass


class Severity(Enum):
    """Error severity levels."""

    INFO = 'info'
    WARNING = 'warning'
    CRITICAL = 'critical'

    @property
    def symbol(self) -> str:
        """Get display symbol for this severity."""
        return {
            Severity.INFO: '[INFO]',
            Severity.WARNING: '[WARNING]',
            Severity.CRITICAL: '[CRITICAL]',
        }[self]

    @property
    def color_code(self) -> str:
        """Get ANSI color code for terminal output."""
        return {
            Severity.INFO: '\033[36m',  # Cyan
            Severity.WARNING: '\033[33m',  # Yellow
            Severity.CRITICAL: '\033[91m',  # Bright Red
        }[self]

    @property
    def reset_code(self) -> str:
        """ANSI reset code."""
        return '\033[0m'

    def format_name(self) -> str:
        """Get formatted severity name."""
        return self.value.upper()

    def colorize(self, text: str) -> str:
        """Colorize text with severity color."""
        return f'{self.color_code}{text}{self.reset_code}'


@dataclass(frozen=True)
class ErrorFlags:
    """Universal error flag handler for any oven type.

    Handles all error operations generically - works with any oven's error codes.
    """

    # The IntFlag enum for this specific oven type
    flag_type: type[IntFlag]

    # Error information: bit_value -> (name, description, severity)
    definitions: Dict[int, Tuple[str, str, Severity]]

    @classmethod
    def from_definitions(
        cls, name: str, definitions: Dict[int, Tuple[str, str, Severity]]
    ) -> 'ErrorFlags':
        """Create an ErrorFlags instance from error definitions.

        Args:
            name: Name for the generated IntFlag enum (e.g., "OvenErrors")
            definitions: Dict mapping bit values to (name, description, severity)
                Example: {
                    1: ("NO_SENSOR", "No sensor detected", Severity.CRITICAL),
                    2: ("LOW_VOLTAGE", "Voltage too low", Severity.WARNING),
                }

        Returns:
            ErrorFlags instance configured for this oven type.
        """
        # Dynamically create IntFlag enum
        flag_dict = {'NONE': 0}
        for bit_value, (error_name, _, _) in definitions.items():
            flag_dict[error_name] = bit_value

        flag_type = IntFlag(name, flag_dict)

        return cls(flag_type=flag_type, definitions=definitions)

    def from_code(self, code: int) -> IntFlag:
        """Parse error code into flag enum."""
        return self.flag_type(code) if code != 0 else self.flag_type(0)

    def list_active(self, code: int) -> List[IntFlag]:
        """Extract individual error flags from error code."""
        if code == 0:
            return []

        flags = self.from_code(code)
        active = []

        for member in self.flag_type:
            if (
                member.value != 0
                and (flags.value & member.value) == member.value
            ):
                active.append(member)

        return active

    def get_descriptions(self, code: int) -> List[str]:
        """Get human-readable descriptions for active errors."""
        active = self.list_active(code)
        descriptions = []

        for flag in active:
            if flag.value in self.definitions:
                _, description, _ = self.definitions[flag.value]
                descriptions.append(description)

        return descriptions

    def get_details(self, code: int) -> List[Dict[str, any]]:
        """Get detailed information about active errors."""
        active = self.list_active(code)
        details = []

        for flag in active:
            if flag.value in self.definitions:
                name, description, severity = self.definitions[flag.value]
                details.append({
                    'name': name,
                    'bit_value': flag.value,
                    'description': description,
                    'severity': severity,
                    'flag': flag,
                })

        return details

    def has_error(self, code: int) -> bool:
        """Check if any error is present."""
        return code != 0

    def has_specific_error(self, code: int, bit_value: int) -> bool:
        """Check if specific error bit is set."""
        return bool(code & bit_value)

    def has_severity(self, code: int, severity: Severity) -> bool:
        """Check if any errors of given severity are present."""
        for detail in self.get_details(code):
            if detail['severity'] == severity:
                return True
        return False

    def has_critical(self, code: int) -> bool:
        """Check if any critical errors are present."""
        return self.has_severity(code, Severity.CRITICAL)

    def has_warnings(self, code: int) -> bool:
        """Check if any warnings are present."""
        return self.has_severity(code, Severity.WARNING)

    def filter_by_severity(
        self, code: int, severity: Severity
    ) -> List[Dict[str, any]]:
        """Filter errors by severity level."""
        return [d for d in self.get_details(code) if d['severity'] == severity]

    def format_report(self, code: int, colorize: bool = True) -> str:
        """Generate formatted error report.

        Args:
            code: Error code to format.
            colorize: Whether to include ANSI color codes.

        Returns:
            Formatted error report string.
        """
        if code == 0:
            return 'No errors detected'

        lines = [f'Active Errors (Code: {code}, Binary: {bin(code)}):']

        for detail in self.get_details(code):
            severity: Severity = detail['severity']

            # Build the error line
            severity_name = severity.format_name()
            error_line = (
                f'  {severity.symbol} [{severity_name}] {detail["name"]}'
            )

            if colorize:
                error_line = severity.colorize(error_line)

            lines.append(error_line)
            lines.append(f'      {detail["description"]}')
            lines.append(f'      Bit {detail["bit_value"]}')

        return '\n'.join(lines)

    def format_summary(self, code: int, colorize: bool = True) -> str:
        """Generate concise error summary.

        Args:
            code: Error code.
            colorize: Whether to include ANSI color codes.

        Returns:
            One-line summary string.
        """
        if code == 0:
            return '✓ No errors'

        details = self.get_details(code)
        critical_count = sum(
            1 for d in details if d['severity'] == Severity.CRITICAL
        )
        warning_count = sum(
            1 for d in details if d['severity'] == Severity.WARNING
        )

        parts = []
        if critical_count:
            text = f'{Severity.CRITICAL.symbol} {critical_count} critical'
            if colorize:
                text = Severity.CRITICAL.colorize(text)
            parts.append(text)

        if warning_count:
            text = f'{Severity.WARNING.symbol} {warning_count} warning(s)'
            if colorize:
                text = Severity.WARNING.colorize(text)
            parts.append(text)

        return ', '.join(parts)

    def create_mask(self, *bit_values: int) -> int:
        """Create bitmask from bit values."""
        mask = 0
        for bit in bit_values:
            mask |= bit
        return mask


# Core protocols


@runtime_checkable
class CoreTemperatureControl(Protocol):
    """Core protocol for basic temperature control.

    This is the minimum interface any temperature controller must implement.
    All temperatures are in degrees Celsius.
    """

    def is_enabled(self) -> bool: ...

    def enable_temperature_control(self) -> bool: ...

    def disable_temperature_control(self) -> bool: ...

    def get_temperature_setpoint(self) -> float:
        """Get current temperature setpoint.

        Returns:
            Temperature setpoint in degrees Celsius.
        """
        ...

    def set_temperature_setpoint(self, temp_celsius: float) -> None:
        """Set target temperature.

        Args:
            temp_celsius: Target temperature in degrees Celsius.
        """
        ...

    def get_actual_temperature(self) -> float:
        """Get actual measured temperature.

        Returns:
            Actual temperature in degrees Celsius.
        """
        ...


@runtime_checkable
class DeviceIdentification(Protocol):
    """Protocol for device identification and information."""

    def get_firmware_version(self) -> str:
        """Get device firmware version.

        Returns:
            Firmware version string.
        """
        ...

    def get_device_id(self) -> str:
        """Get unique device identifier.

        Returns:
            Device identifier (UUID, serial number, etc.).
        """
        ...


@runtime_checkable
class ErrorReporting(Protocol):
    """Protocol for error reporting and management.

    Error codes are integers representing bitfield error states.
    Use the ErrorFlags class to interpret error codes.
    """

    def get_error_code(self) -> int:
        """Get current error code.

        Returns:
            Integer error code representing active error flags.
        """
        ...

    def has_errors(self) -> bool:
        """Check if any errors are present.

        Returns:
            True if any errors are active, False otherwise.
        """
        ...

    def clear_errors(self) -> None:
        """Clear/reset all error conditions."""
        ...


# Standard feature protocols


@runtime_checkable
class PIDConfiguration(Protocol):
    """Protocol for PID controller parameter configuration.

    PID parameters are handled as a group since they are typically
    tuned together and interdependent.
    """

    def get_pid_parameters(self) -> Tuple[float, float, float]:
        """Get PID controller parameters.

        Returns:
            Tuple of (P, I, D) parameters.
        """
        ...

    def set_pid_parameters(self, p: float, i: float, d: float) -> None:
        """Set PID controller parameters.

        Args:
            p: Proportional gain.
            i: Integral gain.
            d: Derivative gain.
        """
        ...


@runtime_checkable
class PowerMonitoring(Protocol):
    """Protocol for monitoring power and electrical parameters."""

    def get_output_power(self) -> float:
        """Get current output power.

        Returns:
            Output power. Units depend on device:
            - Percentage (0-100) for some controllers
            - Milliamps for TEC controllers
            - Watts for resistive heaters
        """
        ...

    def get_supply_voltage(self) -> float:
        """Get supply voltage.

        Returns:
            Supply voltage in volts.
        """
        ...


# Advanced/optional protocols


@runtime_checkable
class ConfigurationPersistence(Protocol):
    """Protocol for saving configuration to non-volatile memory."""

    def save_configuration(self) -> None:
        """Save current configuration to non-volatile memory.

        Persists all settings so they survive power cycles.
        """
        ...


@runtime_checkable
class TemperatureStabilityMonitoring(Protocol):
    """Protocol for monitoring temperature stability."""

    def get_temperature_stability_status(self) -> bool:
        """Get temperature stability status.

        Returns:
            True if temperature is stable within configured window.
        """
        ...

    def get_stability_window(self) -> Tuple[float, float]:
        """Get temperature stability window.

        Returns:
            Tuple of (low_threshold, high_threshold) in degrees Celsius
            relative to setpoint.
        """
        ...

    def set_stability_window(self, low: float, high: float) -> None:
        """Set temperature stability window.

        Args:
            low: Low threshold in degrees Celsius (negative offset from setpoint).
            high: High threshold in degrees Celsius (positive offset from setpoint).
        """
        ...


@runtime_checkable
class ErrorMasking(Protocol):
    """Protocol for masking/ignoring specific errors.

    Allows selective suppression of error conditions.
    Primarily for MTD1020T controllers.
    """

    def get_masked_errors(self) -> int:
        """Get currently masked error flags.

        Returns:
            Bitmask of masked error flags.
        """
        ...

    def mask_error(self, error_bit: int) -> None:
        """Mask (ignore) a specific error.

        Args:
            error_bit: Error bit value to mask.
        """
        ...

    def unmask_error(self, error_bit: int) -> None:
        """Unmask (re-enable) a specific error.

        Args:
            error_bit: Error bit value to unmask.
        """
        ...

    def unmask_all_errors(self) -> None:
        """Unmask all errors (restore default safe state)."""
        ...


@runtime_checkable
class AlarmManagement(Protocol):
    """Protocol for temperature alarm management.

    Primarily for OC3 oven controllers.
    """

    def get_alarm_low(self) -> float:
        """Get low temperature alarm threshold.

        Returns:
            Low alarm threshold in degrees Celsius.
        """
        ...

    def set_alarm_low(self, temp_celsius: float) -> None:
        """Set low temperature alarm threshold.

        Args:
            temp_celsius: Low alarm threshold in degrees Celsius.
        """
        ...

    def get_alarm_high(self) -> float:
        """Get high temperature alarm threshold.

        Returns:
            High alarm threshold in degrees Celsius.
        """
        ...

    def set_alarm_high(self, temp_celsius: float) -> None:
        """Set high temperature alarm threshold.

        Args:
            temp_celsius: High alarm threshold in degrees Celsius.
        """
        ...

    def get_alarm_status(self) -> bool:
        """Get current alarm status.

        Returns:
            True if alarm is active, False otherwise.
        """
        ...


@runtime_checkable
class RampControl(Protocol):
    """Protocol for temperature ramping control.

    Allows controlled temperature change rates.
    Primarily for OC3 oven controllers.
    """

    def get_ramp_rate(self) -> float:
        """Get temperature ramp rate.

        Returns:
            Ramp rate in degrees Celsius per minute.
        """
        ...

    def set_ramp_rate(self, rate_deg_per_min: float) -> None:
        """Set temperature ramp rate.

        Args:
            rate_deg_per_min: Ramp rate in degrees Celsius per minute.
        """
        ...


@runtime_checkable
class TemperatureCycling(Protocol):
    """Protocol for temperature cycling operations.

    Supports automated temperature cycling between setpoints.
    Primarily for OC3 oven controllers.
    """

    def get_cycle_count(self) -> int:
        """Get number of completed temperature cycles.

        Returns:
            Number of completed cycles.
        """
        ...

    def is_cycling(self) -> bool:
        """Check if temperature cycling is active.

        Returns:
            True if cycling is active, False otherwise.
        """
        ...


@runtime_checkable
class CurrentLimitControl(Protocol):
    """Protocol for TEC current limiting.

    Controls maximum current delivered to thermoelectric cooler.
    Primarily for MTD1020T controllers.
    """

    def get_current_limit(self) -> float:
        """Get TEC current limit.

        Returns:
            Current limit in amperes.
        """
        ...

    def set_current_limit(self, limit_amps: float) -> None:
        """Set TEC current limit.

        Args:
            limit_amps: Current limit in amperes.
        """
        ...

    def get_actual_current(self) -> Tuple[float, str]:
        """Get actual TEC current and direction.

        Returns:
            Tuple of (current_amps, mode) where mode is "heating" or "cooling".
        """
        ...


@runtime_checkable
class ActualVoltageMonitoring(Protocol):
    """Protocol for monitoring actual output voltage.

    Separate from supply voltage - this is the voltage across the load.
    Primarily for MTD1020T controllers.
    """

    def get_actual_voltage(self) -> float:
        """Get actual output voltage.

        Returns:
            Output voltage in volts.
        """
        ...


# Composite protocols


@runtime_checkable
class StandardTemperatureController(
    CoreTemperatureControl,
    DeviceIdentification,
    ErrorReporting,
    PIDConfiguration,
    Protocol,
):
    """Standard temperature controller with common features.

    Combines core functionality with PID tuning and error reporting.
    """

    pass


@runtime_checkable
class FullFeaturedTEC(
    StandardTemperatureController,
    PowerMonitoring,
    ConfigurationPersistence,
    ErrorMasking,
    CurrentLimitControl,
    ActualVoltageMonitoring,
    Protocol,
):
    """Full-featured TEC controller protocol.

    Includes all features typically found in precision TEC controllers
    like the MTD1020T.
    """

    pass


@runtime_checkable
class FullFeaturedOven(
    StandardTemperatureController,
    PowerMonitoring,
    AlarmManagement,
    RampControl,
    TemperatureCycling,
    TemperatureStabilityMonitoring,
    Protocol,
):
    """Full-featured oven controller protocol.

    Includes all features typically found in laboratory ovens
    like the OC3.
    """

    pass
