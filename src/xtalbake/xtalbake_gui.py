import sys
from PyQt6 import uic
from PyQt6.QtWidgets import (
    QMainWindow,
    QGridLayout,
    QWidget,
    QLabel,
    QDoubleSpinBox,
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import numpy as np
from collections import deque
from datetime import datetime
from typing import Optional

# Import protocols
from ._tec_protocol import (
    CoreTemperatureControl,
    DeviceIdentification,
    ErrorReporting,
    PIDConfiguration,
    ConfigurationPersistence,
    AlarmManagement,
    RampControl,
    CurrentLimitControl,
    TemperatureStabilityMonitoring,
)
from .device_discovery import discover, create_controller

# Handle UI file loading from package resources
if sys.version_info >= (3, 9):
    from importlib.resources import files

    def get_ui_path(ui_filename):
        """Get path to UI file using importlib.resources"""
        return str(files('xtalbake.ui').joinpath(ui_filename))

else:
    import pkg_resources

    def get_ui_path(ui_filename):
        """Get path to UI file using pkg_resources"""
        return pkg_resources.resource_filename('xtalbake.ui', ui_filename)


matplotlib.use('Qt5Agg')

plot_colors = {
    'temperature': '#E76F51',
    'temp_deviation': '#F4A261',
    'current': '#2A9D8F',
    'power': '#264653',
}


class MatplotlibCanvas(FigureCanvasQTAgg):
    """Matplotlib canvas for embedding in Qt application."""

    def __init__(self, parent=None, width=8, height=4, dpi=100):
        self.fig = Figure(
            figsize=(width, height),
            dpi=dpi,
            facecolor='none',
        )
        self.axes = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)

        self.setStyleSheet('background: transparent;')
        self.axes.set_facecolor('none')
        self.axes.spines['top'].set_visible(False)
        self.axes.spines['right'].set_visible(False)


class xtalbakeGUI(QMainWindow):
    """
    Universal GUI for any temperature controller implementing our protocols.
    """

    def __init__(self, controller_factory=None, discovery_function=None):
        """Initialize GUI.

        Args:
            controller_factory: Optional factory function that takes a port
                                string and returns a controller instance. If
                                None, user must provide it later.
            discovery_function: Optional function that returns list of
                              (port, description, detected_type) tuples.
                              If None, device discovery won't be available.
        """
        super().__init__()

        # Load the UI file
        ui_path = get_ui_path('xtalbake.ui')
        uic.loadUi(ui_path, self)

        self.controller: Optional[CoreTemperatureControl] = None
        self.controller_factory = controller_factory
        self.discovery_function = discovery_function
        self.is_connected = False

        # Store discovered devices
        self.discovered_devices = []

        # Data storage for plotting (60 seconds at ~250ms update rate)
        self.max_points = 60
        self.time_data = deque(maxlen=self.max_points)
        self.temp_data = deque(maxlen=self.max_points)
        self.temp_dev_data = deque(maxlen=self.max_points)
        self.power_data = deque(maxlen=self.max_points)  # Current or power %
        self.start_time = None

        # Setup matplotlib canvas (must be done after UI is loaded)
        self.setup_chart()

        # Connect UI signals
        self.connect_signals()

        # Setup status bar labels (need to add them to the status bar from UI)
        self.setup_status_bar()

        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_values)

        # Do initial device scan if discovery function provided
        if self.discovery_function:
            self.refresh_devices()

    def setup_chart(self):
        """Setup matplotlib canvas in the chart placeholder."""
        # Create the matplotlib canvas
        self.canvas = MatplotlibCanvas(self, width=8, height=4, dpi=100)

        # Replace the placeholder widget with the canvas
        chart_layout = self.chartCanvasPlaceholder.parent().layout()
        chart_layout.replaceWidget(self.chartCanvasPlaceholder, self.canvas)
        self.chartCanvasPlaceholder.deleteLater()

        # Apply styling to legend buttons
        self.showTempCheck.setStyleSheet(f"""
            QPushButton {{
                border: 1px solid palette(mid);
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QPushButton:checked {{
                border: 2px solid {plot_colors['temperature']};
            }}
        """)

        self.showDevCheck.setStyleSheet(f"""
            QPushButton {{
                border: 1px solid palette(mid);
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QPushButton:checked {{
                border: 2px solid {plot_colors['temp_deviation']};
            }}
        """)

        self.showPowerCheck.setStyleSheet(f"""
            QPushButton {{
                border: 1px solid palette(mid);
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QPushButton:checked {{
                border: 2px solid {plot_colors['current']};
            }}
        """)

    def setup_status_bar(self):
        """Setup status bar with labels."""
        self.status_label = QLabel('Disconnected')
        self.error_label = QLabel('')
        self.error_label.setStyleSheet('color: red; font-weight: bold;')
        self.time_label = QLabel('')

        self.statusbar.addWidget(self.status_label)
        self.statusbar.addWidget(self.error_label, 1)
        self.statusbar.addPermanentWidget(self.time_label)

    def connect_signals(self):
        """Connect UI signals to handlers."""
        # Connection buttons
        self.refreshBtn.clicked.connect(self.refresh_devices)
        self.connectBtn.clicked.connect(self.connect_device)
        self.disconnectBtn.clicked.connect(self.disconnect_device)

        # Core control buttons
        self.applyTempButton.clicked.connect(self.apply_temp_change)
        self.enableControlButton.clicked.connect(self.enable_control)
        self.disableControlButton.clicked.connect(self.disable_control)

        # Function buttons
        self.readBtn.clicked.connect(self.read_from_device)
        self.saveBtn.clicked.connect(self.save_to_device)
        self.errorReportBtn.clicked.connect(self.show_error_report)

        # Chart legend toggles
        self.showTempCheck.clicked.connect(self.update_plot)
        self.showDevCheck.clicked.connect(self.update_plot)
        self.showPowerCheck.clicked.connect(self.update_plot)

    def enable_control(self):
        """Enable temperature control."""
        if self.controller:
            self.controller.enable_temperature_control()
            self.enableControlButton.setEnabled(False)
            self.disableControlButton.setEnabled(True)

    def disable_control(self):
        """Disable temperature control."""
        if self.controller:
            self.controller.disable_temperature_control()
            self.enableControlButton.setEnabled(True)
            self.disableControlButton.setEnabled(False)

    def _update_button_states(self):
        """Update button states based on controller status."""
        if self.controller:
            enabled = self.controller.is_enabled()
            self.enableControlButton.setEnabled(not enabled)
            self.disableControlButton.setEnabled(enabled)

    def create_pid_tab_content(self):
        """Create PID parameter controls for tab."""
        widget = QWidget()
        layout = QGridLayout()

        row = 0

        # P Share
        layout.addWidget(QLabel('P (Proportional):'), row, 0)
        self.p_share = QDoubleSpinBox()
        self.p_share.setRange(0, 100000)
        self.p_share.setValue(1.0)
        self.p_share.setDecimals(3)
        self.p_share.valueChanged.connect(self.on_pid_changed)
        layout.addWidget(self.p_share, row, 1)
        row += 1

        # I Share
        layout.addWidget(QLabel('I (Integral):'), row, 0)
        self.i_share = QDoubleSpinBox()
        self.i_share.setRange(0, 100000)
        self.i_share.setValue(0.2)
        self.i_share.setDecimals(3)
        self.i_share.valueChanged.connect(self.on_pid_changed)
        layout.addWidget(self.i_share, row, 1)
        row += 1

        # D Share
        layout.addWidget(QLabel('D (Derivative):'), row, 0)
        self.d_share = QDoubleSpinBox()
        self.d_share.setRange(0, 100000)
        self.d_share.setValue(0.1)
        self.d_share.setDecimals(3)
        self.d_share.valueChanged.connect(self.on_pid_changed)
        layout.addWidget(self.d_share, row, 1)

        widget.setLayout(layout)
        return widget

    def create_limits_alarms_tab_content(self):
        """Create limits and alarms controls for tab."""
        widget = QWidget()
        layout = QGridLayout()

        row = 0

        # Current limit (MTD1020T)
        if isinstance(self.controller, CurrentLimitControl):
            layout.addWidget(QLabel('Current Limit:'), row, 0)
            self.current_limit = QDoubleSpinBox()
            self.current_limit.setRange(0.2, 2.0)
            self.current_limit.setValue(2.0)
            self.current_limit.setSuffix(' A')
            self.current_limit.setDecimals(3)
            self.current_limit.valueChanged.connect(
                self.on_current_limit_changed
            )
            layout.addWidget(self.current_limit, row, 1)
            row += 1

        # Alarms (OC3)
        if isinstance(self.controller, AlarmManagement):
            layout.addWidget(QLabel('Low Alarm:'), row, 0)
            self.alarm_low = QDoubleSpinBox()
            self.alarm_low.setRange(-50.0, 150.0)
            self.alarm_low.setValue(20.0)
            self.alarm_low.setSuffix(' °C')
            self.alarm_low.setDecimals(1)
            self.alarm_low.valueChanged.connect(self.on_alarm_low_changed)
            layout.addWidget(self.alarm_low, row, 1)
            row += 1

            layout.addWidget(QLabel('High Alarm:'), row, 0)
            self.alarm_high = QDoubleSpinBox()
            self.alarm_high.setRange(-50.0, 150.0)
            self.alarm_high.setValue(40.0)
            self.alarm_high.setSuffix(' °C')
            self.alarm_high.setDecimals(1)
            self.alarm_high.valueChanged.connect(self.on_alarm_high_changed)
            layout.addWidget(self.alarm_high, row, 1)

        widget.setLayout(layout)
        return widget

    def create_temperature_control_tab_content(self):
        """Create temperature control features (ramp, cycling) for tab."""
        widget = QWidget()
        layout = QGridLayout()

        row = 0

        # Ramp control (OC3)
        if isinstance(self.controller, RampControl):
            layout.addWidget(QLabel('Ramp Rate:'), row, 0)
            self.ramp_rate = QDoubleSpinBox()
            self.ramp_rate.setRange(0.0, 100.0)
            self.ramp_rate.setValue(0.0)
            self.ramp_rate.setSuffix(' °C/min')
            self.ramp_rate.setDecimals(2)
            self.ramp_rate.valueChanged.connect(self.on_ramp_rate_changed)
            layout.addWidget(self.ramp_rate, row, 1)

        widget.setLayout(layout)
        return widget

    def create_stability_tab_content(self):
        """Create stability monitoring controls for tab."""
        widget = QWidget()
        layout = QGridLayout()

        layout.addWidget(QLabel('Stability Window:'), 0, 0)
        self.stability_window = QDoubleSpinBox()
        self.stability_window.setRange(0.001, 10.0)
        self.stability_window.setValue(0.1)
        self.stability_window.setPrefix('± ')
        self.stability_window.setSuffix(' °C')
        self.stability_window.setDecimals(3)
        self.stability_window.valueChanged.connect(self.on_stability_changed)
        layout.addWidget(self.stability_window, 0, 1)

        self.stability_status = QLabel('--')
        self.stability_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(QLabel('Status:'), 1, 0)
        layout.addWidget(self.stability_status, 1, 1)

        widget.setLayout(layout)
        return widget

    def setup_optional_controls(self):
        """Setup feature tabs based on controller capabilities."""
        if not self.controller:
            return

        # Clear any existing tabs
        self.featureTabWidget.clear()

        # Tab 1: PID Tuning (most common, almost all devices)
        if isinstance(self.controller, PIDConfiguration):
            pid_widget = self.create_pid_tab_content()
            self.featureTabWidget.addTab(pid_widget, 'PID Tuning')

        # Tab 2: Limits & Alarms (device-specific)
        if isinstance(self.controller, (CurrentLimitControl, AlarmManagement)):
            limits_widget = self.create_limits_alarms_tab_content()
            self.featureTabWidget.addTab(limits_widget, 'Limits & Alarms')

        # Tab 3: Temperature Control (ramp, cycling - OC3 specific)
        if isinstance(self.controller, RampControl):
            temp_control_widget = self.create_temperature_control_tab_content()
            self.featureTabWidget.addTab(
                temp_control_widget, 'Temperature Control'
            )

        # Tab 4: Stability & Monitoring
        if isinstance(self.controller, TemperatureStabilityMonitoring):
            stability_widget = self.create_stability_tab_content()
            self.featureTabWidget.addTab(stability_widget, 'Stability')

        # Update save button text based on persistence support
        if isinstance(self.controller, ConfigurationPersistence):
            self.saveBtn.setText('Save Settings to Non-Volatile Memory')
            self.saveBtn.setEnabled(True)
        else:
            self.saveBtn.setText('Save Not Supported')
            self.saveBtn.setEnabled(False)

    def refresh_devices(self):
        """Scan for and populate the device dropdown."""
        if not self.discovery_function:
            # If no discovery function, add a manual entry option
            self.deviceCombo.clear()
            self.deviceCombo.addItem('Manual: /dev/ttyUSB0', '/dev/ttyUSB0')
            self.deviceCombo.setEditable(True)
            self.status_label.setText(
                'Device discovery not available (enter port manually)'
            )
            return

        try:
            # Clear current items
            self.deviceCombo.clear()

            # Discover devices
            self.status_label.setText('Scanning for devices...')
            devices = self.discovery_function()
            self.discovered_devices = devices

            if not devices:
                self.deviceCombo.addItem('No devices found', None)
                self.status_label.setText('No devices found')
                self.connectBtn.setEnabled(False)
            else:
                # Populate dropdown with discovered devices
                for port, description, detected_type in devices:
                    # Format the display text
                    if detected_type:
                        display_text = (
                            f'[{detected_type}] {port} - {description}'
                        )
                    else:
                        display_text = f'[Unknown] {port} - {description}'

                    # Store port as item data
                    self.deviceCombo.addItem(display_text, port)

                self.status_label.setText(f'Found {len(devices)} device(s)')
                self.connectBtn.setEnabled(True)

        except Exception as e:
            self.error_label.setText(f'Scan error: {str(e)}')
            self.status_label.setText('Device scan failed')
            # Add manual entry fallback
            self.deviceCombo.clear()
            self.deviceCombo.addItem('Manual: /dev/ttyUSB0', '/dev/ttyUSB0')
            self.deviceCombo.setEditable(True)

    def connect_device(self):
        """Connect to the temperature controller."""
        if not self.controller_factory:
            self.error_label.setText('Error: No controller factory configured')
            return

        # Get selected port from dropdown
        port = self.deviceCombo.currentData()

        if not port:
            self.error_label.setText('Error: No valid device selected')
            return

        try:
            # Use factory to create appropriate controller
            self.controller = self.controller_factory(port)
            self.is_connected = True

            # Update UI
            self.connectBtn.setEnabled(False)
            self.disconnectBtn.setEnabled(True)
            self.deviceCombo.setEnabled(False)
            self.refreshBtn.setEnabled(False)
            self.readBtn.setEnabled(True)
            self.errorReportBtn.setEnabled(True)

            # Enable feature tab widget and core controls
            self.featureTabWidget.setEnabled(True)
            self.coreParametersGroupBox.setEnabled(True)

            # Setup optional controls based on capabilities
            self.setup_optional_controls()

            # Read device info
            if isinstance(self.controller, DeviceIdentification):
                version = self.controller.get_firmware_version()
                device_id = self.controller.get_device_id()
                self.firmwareLabel.setText(version)
                self.deviceIdLabel.setText(device_id)

            # Read current settings
            self.read_from_device()

            # Start update timer
            self.start_time = datetime.now()
            self.update_timer.start(250)  # Update every 250ms

            self.status_label.setText(f'Connected to {port}')
            self.error_label.setText('')
            self._update_button_states()

        except Exception as e:
            self.status_label.setText('Connection Failed')
            self.error_label.setText(f'Error: {str(e)}')
            self.is_connected = False
            self.controller = None

    def disconnect_device(self):
        """Disconnect from the device."""
        if self.controller:
            self.update_timer.stop()

            # Close connection if controller supports context manager
            if hasattr(self.controller, '__exit__'):
                self.controller.__exit__(None, None, None)

            self.controller = None
            self.is_connected = False

        # Reset UI
        self.connectBtn.setEnabled(True)
        self.disconnectBtn.setEnabled(False)
        self.deviceCombo.setEnabled(True)
        self.refreshBtn.setEnabled(True)
        self.readBtn.setEnabled(False)
        self.saveBtn.setEnabled(False)
        self.errorReportBtn.setEnabled(False)

        # Disable feature tab widget and core controls
        self.featureTabWidget.setEnabled(False)
        self.coreParametersGroupBox.setEnabled(False)

        self.firmwareLabel.setText('--')
        self.deviceIdLabel.setText('--')
        self.status_label.setText('Disconnected')
        self.error_label.setText('')

        # Clear plot data
        self.time_data.clear()
        self.temp_data.clear()
        self.temp_dev_data.clear()
        self.power_data.clear()
        self.update_plot()

    def read_from_device(self):
        """Read current settings from device."""
        if not self.is_connected or not self.controller:
            return

        try:
            # Block signals to avoid triggering updates during read
            self.tempSetpoint.blockSignals(True)

            # Core temperature
            temp = self.controller.get_temperature_setpoint()
            self.tempSetpoint.setValue(temp)

            # PID parameters if available
            if isinstance(self.controller, PIDConfiguration):
                self.p_share.blockSignals(True)
                self.i_share.blockSignals(True)
                self.d_share.blockSignals(True)

                p, i, d = self.controller.get_pid_parameters()
                self.p_share.setValue(p)
                self.i_share.setValue(i)
                self.d_share.setValue(d)

                self.p_share.blockSignals(False)
                self.i_share.blockSignals(False)
                self.d_share.blockSignals(False)

            # Current limit if available
            if isinstance(self.controller, CurrentLimitControl):
                self.current_limit.blockSignals(True)
                limit = self.controller.get_current_limit()
                self.current_limit.setValue(limit)
                self.current_limit.blockSignals(False)

            # Alarm thresholds if available
            if isinstance(self.controller, AlarmManagement):
                self.alarm_low.blockSignals(True)
                self.alarm_high.blockSignals(True)

                low = self.controller.get_alarm_low()
                high = self.controller.get_alarm_high()
                self.alarm_low.setValue(low)
                self.alarm_high.setValue(high)

                self.alarm_low.blockSignals(False)
                self.alarm_high.blockSignals(False)

            # Ramp rate if available
            if isinstance(self.controller, RampControl):
                self.ramp_rate.blockSignals(True)
                rate = self.controller.get_ramp_rate()
                self.ramp_rate.setValue(rate)
                self.ramp_rate.blockSignals(False)

            # Stability window if available
            if isinstance(self.controller, TemperatureStabilityMonitoring):
                self.stability_window.blockSignals(True)
                low, high = self.controller.get_stability_window()
                # Use the larger of the two
                window = max(abs(low), abs(high))
                self.stability_window.setValue(window)
                self.stability_window.blockSignals(False)

            self.tempSetpoint.blockSignals(False)
            self.status_label.setText('Settings read from device')

        except Exception as e:
            self.error_label.setText(f'Read error: {str(e)}')

    def save_to_device(self):
        """Save current settings to device non-volatile memory."""
        if not self.is_connected or not self.controller:
            return

        if not isinstance(self.controller, ConfigurationPersistence):
            self.error_label.setText(
                'Device does not support configuration save'
            )
            return

        try:
            self.controller.save_configuration()
            self.status_label.setText('Settings saved to device memory')
        except Exception as e:
            self.error_label.setText(f'Save error: {str(e)}')

    def show_error_report(self):
        """Display detailed error report."""
        if not self.is_connected or not self.controller:
            return

        if not isinstance(self.controller, ErrorReporting):
            return

        try:
            # Get error report if controller has this method
            if hasattr(self.controller, 'get_error_report'):
                report = self.controller.get_error_report(colorize=False)
                # Show in a message box or dialog
                from PyQt6.QtWidgets import QMessageBox

                msg = QMessageBox()
                msg.setWindowTitle('Error Report')
                msg.setText(report)
                msg.setFont(QFont('Courier', 10))
                msg.exec()
            else:
                error_code = self.controller.get_error_code()
                self.status_label.setText(f'Error code: {error_code}')

        except Exception as e:
            self.error_label.setText(f'Error reading report: {str(e)}')

    def update_values(self):
        """Update displayed values from device."""
        if not self.is_connected or not self.controller:
            return

        actual_temp = self.controller.get_actual_temperature()
        set_temp = self.controller.get_temperature_setpoint()

        # Update temperature displays
        self.tempValue.setText(f'{actual_temp:.3f}')
        temp_dev = actual_temp - set_temp
        self.tempDevValue.setText(f'{temp_dev:.3f}')

        # Update power/current display
        power = self.controller.get_output_power()
        self.powerValue.setText(f'{power:.3f}')

        # Check if it's current (A) or percentage
        if isinstance(self.controller, CurrentLimitControl):
            self.powerUnit.setText('A')
        else:
            self.powerUnit.setText('%')

        voltage = self.controller.get_supply_voltage()
        self.voltageValue.setText(f'{voltage:.2f}')

        if self.controller.has_errors():
            error_code = self.controller.get_error_code()
            self.error_label.setText(f'Error code: {error_code}')
        else:
            self.error_label.setText('')

        # Update stability status if widget exists
        if hasattr(self, 'stability_status'):
            is_stable = self.controller.get_temperature_stability_status()
            self.stability_status.setText('Stable' if is_stable else 'Settling')
            self.stability_status.setStyleSheet(
                'color: green; font-weight: bold;'
                if is_stable
                else 'color: orange;'
            )

        # Update time
        self.time_label.setText(datetime.now().strftime('%Y-%m-%d %H:%M:%S'))

        # Add to plot data
        if self.start_time:
            elapsed = (datetime.now() - self.start_time).total_seconds()
            self.time_data.append(elapsed)
            self.temp_data.append(actual_temp)
            self.temp_dev_data.append(temp_dev)
            self.power_data.append(power)

            self.update_plot()

    def update_plot(self):
        """Update the real-time plot."""
        if len(self.time_data) == 0:
            return

        # Clear and recreate axes
        self.canvas.fig.clear()
        ax1 = self.canvas.fig.add_subplot(111)
        ax1.set_facecolor('none')
        ax1.spines['top'].set_visible(False)
        ax1.spines['right'].set_visible(False)

        # Twin axis for power/current
        ax2 = ax1.twinx()
        ax2.spines['top'].set_visible(False)

        time_array = np.array(self.time_data)

        # Plot temperature
        if self.showTempCheck.isChecked() and len(self.temp_data) > 0:
            ax1.plot(
                time_array,
                np.array(self.temp_data),
                color=plot_colors['temperature'],
                linewidth=2,
                label='Temperature',
            )

        # Plot deviation
        if self.showDevCheck.isChecked() and len(self.temp_dev_data) > 0:
            ax1.plot(
                time_array,
                np.array(self.temp_dev_data),
                color=plot_colors['temp_deviation'],
                linewidth=2,
                label='Deviation',
            )

        # Plot power/current
        if self.showPowerCheck.isChecked() and len(self.power_data) > 0:
            ax2.plot(
                time_array,
                np.array(self.power_data),
                color=plot_colors['current'],
                linewidth=2,
                label='Power/Current',
            )

        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Temperature [°C] / Deviation [K]')

        # Label second axis appropriately
        if self.controller and isinstance(self.controller, CurrentLimitControl):
            ax2.set_ylabel('Current [A]')
        else:
            ax2.set_ylabel('Power [%]')

        ax1.grid(True, alpha=0.3)
        self.canvas.draw()

    # Parameter change handlers
    def apply_temp_change(self):
        """Apply temperature setpoint change."""
        value = self.tempSetpoint.value()
        if self.is_connected and self.controller:
            self.controller.set_temperature_setpoint(value)

    def on_pid_changed(self, value):
        """Handle PID parameter change."""
        if self.is_connected and self.controller:
            if not isinstance(self.controller, PIDConfiguration):
                return
            try:
                p = self.p_share.value()
                i = self.i_share.value()
                d = self.d_share.value()
                self.controller.set_pid_parameters(p, i, d)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_current_limit_changed(self, value):
        """Handle current limit change."""
        if self.is_connected and self.controller:
            if not isinstance(self.controller, CurrentLimitControl):
                return
            try:
                self.controller.set_current_limit(value)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_alarm_low_changed(self, value):
        """Handle low alarm threshold change."""
        if self.is_connected and self.controller:
            if not isinstance(self.controller, AlarmManagement):
                return
            try:
                self.controller.set_alarm_low(value)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_alarm_high_changed(self, value):
        """Handle high alarm threshold change."""
        if self.is_connected and self.controller:
            if not isinstance(self.controller, AlarmManagement):
                return
            try:
                self.controller.set_alarm_high(value)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_ramp_rate_changed(self, value):
        """Handle ramp rate change."""
        if self.is_connected and self.controller:
            if not isinstance(self.controller, RampControl):
                return
            try:
                self.controller.set_ramp_rate(value)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_stability_changed(self, value):
        """Handle stability window change."""
        if self.is_connected and self.controller:
            if not isinstance(self.controller, TemperatureStabilityMonitoring):
                return
            try:
                # Set symmetric window
                self.controller.set_stability_window(-value, value)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')


def main():
    """Main application entry point."""
    try:
        from PyQt6.QtWidgets import QApplication
        import matplotlib
    except ImportError as e:
        print('Error: GUI dependencies not installed.')
        print()
        print('The xtalbake GUI requires PyQt6 and matplotlib.')
        print('Install them with:')
        print()
        print('    pip install xtalbake[gui]')
        print(
            '    pip install "xtalbake[gui] @ git+https://github.com/Peter-Barrow/xtalbake.git"'
        )
        print()
        print('Or install the missing packages directly:')
        print('    pip install pyqt6 matplotlib')
        raise SystemExit(1) from e

    app = QApplication(sys.argv)

    window = xtalbakeGUI(
        controller_factory=create_controller,
        discovery_function=discover,
    )
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
