# xtalbake

A Python package and GUI tool for controlling and monitoring the MTD1020T TEC (Thermoelectric Cooler) Controller over a serial connection.  It provides functions for managing temperature control, PID parameters, safety settings, and includes a PyQt-based interface for interactive use.

![gui](screenshot.png)

## Features

- Query system information:
  - Firmware version (`version`)
  - Device UUID (`uuid`)
  - Error register and active errors (`errors`)
- Temperature control:
  - Set and read target temperature (`temperature_set_point`)
  - Monitor actual temperature (`actual_temperature`)
  - Configure temperature window and delay
- TEC current management:
  - Set current limit (`current_limit`)
  - Read actual current and voltage
  - Monitor heating/cooling mode
- PID tuning:
  - Configure P, I, D shares
  - Set cycle time
  - Loop test parameters (critical gain and period)
- Safety features:
  - Mask/unmask error conditions
  - Reset error register
- Configuration persistence:
  - Save settings to non-volatile memory (`save_to_memory`)
- High-level `MTD1020T` class with property-based interface
- Context manager support for automatic connection handling

## Installation

Clone and install directly from GitHub:

```bash
pip install git+https://github.com/Peter-Barrow/xtalbake.git
```

### Linux

On Linux you made to do the following:
- Install libftdi (`dnf install libftdi`)
- You may also need to associate device with ftdio_sio, something like `echo "<VID> <PID>" | sudo tee /srv/bus/usb-serial/ftdio_sio/new_id`, you can find `<VID> <PID>` from `lsusb`

- Device appearing with `lsusb` but not accessible?
`lsusb -t -v` -> check if the driver is loaded
if not load the `ftdi_sio` driver
`sudo modprobe ftdi_sio`
then run
`echo "<VID> <PID>" | sudo tee /srv/bus/usb-serial/ftdio_sio/new_id`



## Usage

### As a library

You can import the package and use the `MTD1020T` class directly:

```python
from xtalbake.mtd1020t import MTD1020T

# Using context manager
with MTD1020T(port="/dev/ttyUSB0") as controller:
    print(controller.version)
    print(controller)  # Detailed status summary
    
    # Set temperature to 25°C
    controller.temperature_set_point = 25000  # in millidegrees
    
    # Configure current limit
    controller.current_limit = 1500  # mA
    
    # Monitor temperature
    set_temp, actual_temp = controller.temperature
    print(f"Target: {set_temp/1000}°C, Actual: {actual_temp/1000}°C")
    
    # Save configuration
    controller.save_to_memory()
```

### As a GUI

To launch the graphical interface:

```bash
xtalbake-ui
```

This will open a window where you can connect to the controller, adjust temperature settings, configure PID parameters, and monitor device status.

## API Reference

### MTD1020T Class

The `MTD1020T` class provides a high-level interface to the device with the following properties and methods:

#### System Information
- `version` - Firmware version string (read-only)
- `uuid` - Device UUID (read-only)
- `errors` - Dictionary of active error flags (read-only)
- `reset_errors()` - Reset the error register

#### Safety & Error Masking
- `masked_errors` - List of currently masked errors (read/write)
- `mask_error(error)` - Add an error to the mask
- `unmask_error(error)` - Remove an error from the mask
- `unmask_all_errors()` - Unmask all errors

#### TEC Current Control
- `current_limit` - TEC current limit in mA (200-2000)
- `actual_current` - Tuple of (current_mA, mode) where mode is 'heating' or 'cooling' (read-only)
- `actual_voltage` - Actual TEC voltage in mV (read-only)

#### Temperature Control
- `temperature_set_point` - Target temperature in millidegrees C (5000-45000)
- `actual_temperature` - Actual temperature in millidegrees C (read-only)
- `temperature` - Tuple of (set_point, actual) temperatures (read-only)
- `temperature_set_point_window` - Temperature window in millikelvin (1-32000)
- `temperature_control_delay` - Delay before status activation in seconds (1-32000)

#### PID Parameters
- `pid_p_share` - Proportional gain in mA/K (0-100000)
- `pid_i_share` - Integral gain in mA/(K·s) (0-100000)
- `pid_d_share` - Derivative gain in (mA·s)/K (0-100000)
- `pid_cycle_time` - PID cycle time in milliseconds (1-1000)

#### Loop Test Parameters
- `loop_test_critical_gain` - Critical gain in mA/K (10-100000)
- `loop_test_critical_period` - Critical period in milliseconds (100-100000)

#### Configuration
- `save_to_memory()` - Save current parameters to non-volatile memory

### Low-Level Serial Commands

For advanced users who need direct serial access, the module provides low-level command functions:

#### Command Structure
Commands are ASCII strings terminated with `\n`. Responses are returned as strings with brackets and whitespace stripped.

**Query format:** `<command>?` (e.g., `m?` for version)  
**Set format:** `<command><value>` (e.g., `T25000` to set 25°C)

#### System Information Commands
- `sys_info_get_version(conn)` - Command: `m?`
- `sys_info_get_uuid(conn)` - Command: `u?`
- `sys_info_get_error_register(conn)` - Command: `E?`
- `sys_info_get_error_flags(conn)` - Returns ErrorFlags enum
- `sys_info_get_active_errors(conn)` - Returns error dictionary
- `sys_info_reset_error_register(conn)` - Command: `c`

#### Safety Bitmask Commands
- `safety_bitmask_set_masked_errors(conn, *errors)` - Command: `S<bitmask>`
- `safety_bitmask_get_masked_errors(conn)` - Command: `S?`
- `safety_bitmask_mask_error(conn, error)` - Add error to mask
- `safety_bitmask_unmask_error(conn, error)` - Remove error from mask
- `safety_bitmask_unmask_all(conn)` - Command: `S0`

#### TEC Control Commands
- `tec_control_set_current_limit(conn, limit_ma)` - Command: `L<value>`
- `tec_control_get_current_limit(conn)` - Command: `L?`
- `tec_control_get_actual_current(conn)` - Command: `A?`
- `tec_control_get_actual_voltage(conn)` - Command: `U?`

#### Temperature Control Commands
- `temperature_control_set_temperature(conn, temp_millidegrees)` - Command: `T<value>`
- `temperature_control_get_set_temperature(conn)` - Command: `T?`
- `temperature_control_get_actual_temperature(conn)` - Command: `Te?`
- `temperature_control_set_window(conn, window_mk)` - Command: `W<value>`
- `temperature_control_get_window(conn)` - Command: `W?`
- `temperature_control_set_delay(conn, delay_sec)` - Command: `d<value>`
- `temperature_control_get_delay(conn)` - Command: `d?`

#### Loop Test Commands
- `loop_test_set_critical_gain(conn, gain)` - Command: `G<value>`
- `loop_test_get_critical_gain(conn)` - Command: `G?`
- `loop_test_set_critical_period(conn, period_ms)` - Command: `O<value>`
- `loop_test_get_critical_period(conn)` - Command: `O?`

#### PID Settings Commands
- `pid_settings_set_p_share(conn, p_value)` - Command: `P<value>`
- `pid_settings_get_p_share(conn)` - Command: `P?`
- `pid_settings_set_i_share(conn, i_value)` - Command: `I<value>`
- `pid_settings_get_i_share(conn)` - Command: `I?`
- `pid_settings_set_d_share(conn, d_value)` - Command: `D<value>`
- `pid_settings_get_d_share(conn)` - Command: `D?`
- `pid_settings_set_cycle_time(conn, cycle_ms)` - Command: `C<value>`
- `pid_settings_get_cycle_time(conn)` - Command: `C?`

#### Configuration Commands
- `save_configuration_to_memory(conn)` - Command: `M`

#### Core Serial Functions
- `write_command(conn, command)` - Write a command with `\n` terminator
- `read_response(conn, timeout)` - Read response until `\n`
- `query(conn, command, timeout)` - Send command and read response

### Error Flags

The `ErrorFlags` enum defines all possible error conditions:

- `ENABLE_PIN_NOT_SET` - Enable pin not set to L (GND)
- `INTERNAL_TEMP_HIGH` - Internal temperature too high
- `THERMAL_LATCH_UP` - TEC current at limit without temperature improvement
- `CYCLING_TIME_SMALL` - Cycling time too small
- `NO_SENSOR` - No sensor detected
- `NO_TEC` - No TEC detected (connection open)
- `TEC_MISPOLED` - TEC mispoled
- `VALUE_OUT_OF_RANGE` - Value out of range
- `INVALID_COMMAND` - Invalid command

## Requirements

* Python 3.10+
* [PyQt6](https://pypi.org/project/PyQt6/)
* [pyserial](https://pypi.org/project/pyserial/)
* [matplotlib](https://pypi.org/project/matplotlib/)
