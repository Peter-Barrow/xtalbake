import sys
from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QDoubleSpinBox,
    QGroupBox,
    QStatusBar,
    QComboBox,
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

        # Store widget references for dynamic show/hide
        self.optional_widgets = {}

        self.init_ui()

        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_values)

        # Do initial device scan if discovery function provided
        if self.discovery_function:
            self.refresh_devices()

    def init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle('xtalbake')
        self.resize(900, 850)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Left column: Chart and core controls
        left_column = QVBoxLayout()
        self.chart_widget = self.create_chart_group()
        left_column.addWidget(self.chart_widget, stretch=3)

        # Core parameters (always shown)
        left_column.addWidget(self.create_core_parameters_group(), stretch=1)

        # Right column: Readouts and connection
        right_column = QVBoxLayout()
        right_column.setAlignment(Qt.AlignmentFlag.AlignTop)

        right_column.addWidget(self.create_readouts_group(), stretch=2)
        right_column.addWidget(self.create_connection_group(), stretch=1)
        right_column.addWidget(self.create_device_info_group(), stretch=1)

        # Optional parameter groups (shown based on capabilities)
        self.optional_params_container = QWidget()
        self.optional_params_layout = QVBoxLayout(
            self.optional_params_container
        )
        self.optional_params_layout.setContentsMargins(0, 0, 0, 0)
        right_column.addWidget(self.optional_params_container, stretch=2)

        right_column.addWidget(self.create_buttons_group(), stretch=1)

        main_layout.addLayout(left_column, stretch=3)
        main_layout.addLayout(right_column, stretch=2)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel('Disconnected')
        self.error_label = QLabel('')
        self.error_label.setStyleSheet('color: red; font-weight: bold;')
        self.time_label = QLabel('')
        self.status_bar.addWidget(self.status_label)
        self.status_bar.addWidget(self.error_label, 1)
        self.status_bar.addPermanentWidget(self.time_label)

    def create_connection_group(self):
        """Create connection control group with device discovery."""
        group = QGroupBox()
        layout = QGridLayout()

        # Device selection row
        layout.addWidget(QLabel('Device:'), 0, 0)

        # Dropdown for device selection
        self.device_combo = QComboBox()
        self.device_combo.setMinimumWidth(200)
        layout.addWidget(self.device_combo, 0, 1)

        # Refresh button
        self.refresh_btn = QPushButton('Scan')
        self.refresh_btn.setMaximumWidth(80)
        self.refresh_btn.clicked.connect(self.refresh_devices)
        self.refresh_btn.setToolTip('Scan for connected devices')
        layout.addWidget(self.refresh_btn, 0, 2)

        # Connection buttons row
        self.connect_btn = QPushButton('Connect')
        self.connect_btn.clicked.connect(self.connect_device)
        layout.addWidget(self.connect_btn, 1, 0, 1, 2)

        self.disconnect_btn = QPushButton('Disconnect')
        self.disconnect_btn.clicked.connect(self.disconnect_device)
        self.disconnect_btn.setEnabled(False)
        layout.addWidget(self.disconnect_btn, 1, 2)

        group.setLayout(layout)
        return group

    def create_device_info_group(self):
        """Create device information group."""
        group = QGroupBox()
        layout = QGridLayout()

        layout.addWidget(QLabel('Firmware Version:'), 0, 0)
        self.firmware_label = QLabel('--')
        layout.addWidget(self.firmware_label, 0, 1)

        layout.addWidget(QLabel('Device ID:'), 1, 0)
        self.device_id_label = QLabel('--')
        self.device_id_label.setFont(QFont('Courier', 8))
        layout.addWidget(self.device_id_label, 1, 1)

        group.setLayout(layout)
        return group

    def create_core_parameters_group(self):
        """Create core temperature control parameters (always present)."""
        group = QGroupBox()
        layout = QGridLayout()

        row = 0

        # Set Temperature
        layout.addWidget(QLabel('Set Temperature:'), row, 0)
        self.temp_setpoint = QDoubleSpinBox()
        self.temp_setpoint.setRange(-50.0, 150.0)  # Generous range
        self.temp_setpoint.setValue(25.0)
        self.temp_setpoint.setSuffix(' °C')
        self.temp_setpoint.setDecimals(3)
        # self.temp_setpoint.valueChanged.connect(self.on_temp_changed)
        layout.addWidget(self.temp_setpoint, row, 1)
        self.enable_control_button = QPushButton('Apply')
        self.enable_control_button.setMaximumWidth(80)
        self.enable_control_button.clicked.connect(self.apply_temp_change)
        layout.addWidget(self.enable_control_button, row, 2)

        self.enable_control_button = QPushButton('Enable')
        self.enable_control_button.setMaximumWidth(80)
        self.enable_control_button.clicked.connect(self.enable_control)
        layout.addWidget(self.enable_control_button, row, 3)

        self.disable_control_button = QPushButton('Disable')
        self.disable_control_button.setMaximumWidth(80)
        self.disable_control_button.clicked.connect(self.disable_control)
        layout.addWidget(self.disable_control_button, row, 4)

        group.setLayout(layout)
        return group

    def enable_control(self):
        self.controller.enable_temperature_control()
        # self._update_button_states()
        self.enable_control_button.setEnabled(False)
        self.disable_control_button.setEnabled(True)

    def disable_control(self):
        self.controller.disable_temperature_control()
        # self._update_button_states()
        self.enable_control_button.setEnabled(True)
        self.disable_control_button.setEnabled(False)

    def _update_button_states(self):
        """Update button states based on controller status"""
        enabled = self.controller.is_enabled()
        self.enable_control_button.setEnabled(not enabled)
        self.disable_control_button.setEnabled(enabled)

    def create_pid_parameters_group(self):
        """Create PID parameter controls."""
        group = QGroupBox()
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

        group.setLayout(layout)
        return group

    def create_current_limit_group(self):
        """Create current limit controls (MTD1020T specific)."""
        group = QGroupBox()
        layout = QGridLayout()

        layout.addWidget(QLabel('Current Limit:'), 0, 0)
        self.current_limit = QDoubleSpinBox()
        self.current_limit.setRange(0.2, 2.0)
        self.current_limit.setValue(2.0)
        self.current_limit.setSuffix(' A')
        self.current_limit.setDecimals(3)
        self.current_limit.valueChanged.connect(self.on_current_limit_changed)
        layout.addWidget(self.current_limit, 0, 1)

        group.setLayout(layout)
        return group

    def create_alarm_group(self):
        """Create alarm controls (OC3 specific)."""
        group = QGroupBox()
        layout = QGridLayout()

        layout.addWidget(QLabel('Low Alarm:'), 0, 0)
        self.alarm_low = QDoubleSpinBox()
        self.alarm_low.setRange(-50.0, 150.0)
        self.alarm_low.setValue(20.0)
        self.alarm_low.setSuffix(' °C')
        self.alarm_low.setDecimals(1)
        self.alarm_low.valueChanged.connect(self.on_alarm_low_changed)
        layout.addWidget(self.alarm_low, 0, 1)

        layout.addWidget(QLabel('High Alarm:'), 1, 0)
        self.alarm_high = QDoubleSpinBox()
        self.alarm_high.setRange(-50.0, 150.0)
        self.alarm_high.setValue(40.0)
        self.alarm_high.setSuffix(' °C')
        self.alarm_high.setDecimals(1)
        self.alarm_high.valueChanged.connect(self.on_alarm_high_changed)
        layout.addWidget(self.alarm_high, 1, 1)

        group.setLayout(layout)
        return group

    def create_ramp_group(self):
        """Create ramp control (OC3 specific)."""
        group = QGroupBox()
        layout = QGridLayout()

        layout.addWidget(QLabel('Ramp Rate:'), 0, 0)
        self.ramp_rate = QDoubleSpinBox()
        self.ramp_rate.setRange(0.0, 100.0)
        self.ramp_rate.setValue(0.0)
        self.ramp_rate.setSuffix(' °C/min')
        self.ramp_rate.setDecimals(2)
        self.ramp_rate.valueChanged.connect(self.on_ramp_rate_changed)
        layout.addWidget(self.ramp_rate, 0, 1)

        group.setLayout(layout)
        return group

    def create_stability_group(self):
        """Create stability monitoring controls."""
        group = QGroupBox()
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

        group.setLayout(layout)
        return group

    def create_buttons_group(self):
        """Create functional buttons group."""
        group = QGroupBox()
        layout = QVBoxLayout()

        self.read_btn = QPushButton('Read Settings from Device')
        self.read_btn.clicked.connect(self.read_from_device)
        self.read_btn.setEnabled(False)
        layout.addWidget(self.read_btn)

        self.save_btn = QPushButton('Save Settings to Device')
        self.save_btn.clicked.connect(self.save_to_device)
        self.save_btn.setEnabled(False)
        layout.addWidget(self.save_btn)

        self.error_report_btn = QPushButton('Show Error Report')
        self.error_report_btn.clicked.connect(self.show_error_report)
        self.error_report_btn.setEnabled(False)
        layout.addWidget(self.error_report_btn)

        group.setLayout(layout)
        return group

    def create_chart_group(self):
        """Create matplotlib chart group."""
        group = QGroupBox()
        layout = QVBoxLayout()

        self.canvas = MatplotlibCanvas(self, width=8, height=4, dpi=100)
        layout.addWidget(self.canvas)

        # Legend checkboxes
        legend_layout = QHBoxLayout()

        self.show_temp_check = QPushButton('Temperature')
        self.show_temp_check.setCheckable(True)
        self.show_temp_check.setChecked(True)
        self.show_temp_check.clicked.connect(self.update_plot)
        self.show_temp_check.setStyleSheet(f"""
            QPushButton {{
                border: 1px solid palette(mid);
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QPushButton:checked {{
                border: 2px solid {plot_colors['temperature']};
            }}
        """)

        self.show_dev_check = QPushButton('Deviation')
        self.show_dev_check.setCheckable(True)
        self.show_dev_check.setChecked(True)
        self.show_dev_check.clicked.connect(self.update_plot)
        self.show_dev_check.setStyleSheet(f"""
            QPushButton {{
                border: 1px solid palette(mid);
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QPushButton:checked {{
                border: 2px solid {plot_colors['temp_deviation']};
            }}
        """)

        self.show_power_check = QPushButton('Power/Current')
        self.show_power_check.setCheckable(True)
        self.show_power_check.setChecked(True)
        self.show_power_check.clicked.connect(self.update_plot)
        self.show_power_check.setStyleSheet(f"""
            QPushButton {{
                border: 1px solid palette(mid);
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QPushButton:checked {{
                border: 2px solid {plot_colors['current']};
            }}
        """)

        legend_layout.addWidget(self.show_temp_check)
        legend_layout.addWidget(self.show_dev_check)
        legend_layout.addWidget(self.show_power_check)
        legend_layout.addStretch()

        layout.addLayout(legend_layout)
        group.setLayout(layout)
        return group

    def create_readouts_group(self):
        """Create real-time readout displays."""
        group = QGroupBox()
        layout = QGridLayout()

        font_large = QFont()
        font_large.setPointSize(20)
        font_large.setBold(True)

        # Temperature
        layout.addWidget(QLabel('Temperature'), 0, 0)
        temp_widget = QWidget()
        temp_layout = QHBoxLayout(temp_widget)
        self.temp_value = QLabel('--')
        self.temp_value.setFont(font_large)
        self.temp_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        temp_layout.addWidget(self.temp_value)
        temp_layout.addWidget(QLabel('°C'))
        layout.addWidget(temp_widget, 1, 0)

        # Temperature Deviation
        layout.addWidget(QLabel('Deviation'), 2, 0)
        dev_widget = QWidget()
        dev_layout = QHBoxLayout(dev_widget)
        self.temp_dev_value = QLabel('--')
        self.temp_dev_value.setFont(font_large)
        self.temp_dev_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        dev_layout.addWidget(self.temp_dev_value)
        self.temp_dev_unit = QLabel('K')
        dev_layout.addWidget(self.temp_dev_unit)
        layout.addWidget(dev_widget, 3, 0)

        # Power/Current
        layout.addWidget(QLabel('Output Power'), 0, 1)
        power_widget = QWidget()
        power_layout = QHBoxLayout(power_widget)
        self.power_value = QLabel('--')
        self.power_value.setFont(font_large)
        self.power_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        power_layout.addWidget(self.power_value)
        self.power_unit = QLabel('A')
        power_layout.addWidget(self.power_unit)
        layout.addWidget(power_widget, 1, 1)

        # Voltage (if available)
        layout.addWidget(QLabel('Voltage'), 2, 1)
        voltage_widget = QWidget()
        voltage_layout = QHBoxLayout(voltage_widget)
        self.voltage_value = QLabel('--')
        self.voltage_value.setFont(font_large)
        self.voltage_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        voltage_layout.addWidget(self.voltage_value)
        voltage_layout.addWidget(QLabel('V'))
        layout.addWidget(voltage_widget, 3, 1)

        group.setLayout(layout)
        return group

    def setup_optional_controls(self):
        """Setup optional controls based on controller capabilities."""
        if not self.controller:
            return

        # Clear existing optional widgets
        while self.optional_params_layout.count():
            item = self.optional_params_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.optional_widgets.clear()

        # Add PID controls if supported
        if isinstance(self.controller, PIDConfiguration):
            pid_group = self.create_pid_parameters_group()
            self.optional_params_layout.addWidget(pid_group)
            self.optional_widgets['pid'] = pid_group

        # Add current limit if supported
        if isinstance(self.controller, CurrentLimitControl):
            current_group = self.create_current_limit_group()
            self.optional_params_layout.addWidget(current_group)
            self.optional_widgets['current_limit'] = current_group

        # Add alarm controls if supported
        if isinstance(self.controller, AlarmManagement):
            alarm_group = self.create_alarm_group()
            self.optional_params_layout.addWidget(alarm_group)
            self.optional_widgets['alarms'] = alarm_group

        # Add ramp controls if supported
        if isinstance(self.controller, RampControl):
            ramp_group = self.create_ramp_group()
            self.optional_params_layout.addWidget(ramp_group)
            self.optional_widgets['ramp'] = ramp_group

        # Add stability monitoring if supported
        if isinstance(self.controller, TemperatureStabilityMonitoring):
            stability_group = self.create_stability_group()
            self.optional_params_layout.addWidget(stability_group)
            self.optional_widgets['stability'] = stability_group

        # Update save button text based on persistence support
        if isinstance(self.controller, ConfigurationPersistence):
            self.save_btn.setText('Save Settings to Non-Volatile Memory')
            self.save_btn.setEnabled(True)
        else:
            self.save_btn.setText('Save Not Supported')
            self.save_btn.setEnabled(False)

    def refresh_devices(self):
        """Scan for and populate the device dropdown."""
        if not self.discovery_function:
            # If no discovery function, add a manual entry option
            self.device_combo.clear()
            self.device_combo.addItem('Manual: /dev/ttyUSB0', '/dev/ttyUSB0')
            self.device_combo.setEditable(True)
            self.status_label.setText(
                'Device discovery not available (enter port manually)'
            )
            return

        try:
            # Clear current items
            self.device_combo.clear()

            # Discover devices
            self.status_label.setText('Scanning for devices...')
            devices = self.discovery_function()
            self.discovered_devices = devices

            if not devices:
                self.device_combo.addItem('No devices found', None)
                self.status_label.setText('No devices found')
                self.connect_btn.setEnabled(False)
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
                    self.device_combo.addItem(display_text, port)

                self.status_label.setText(f'Found {len(devices)} device(s)')
                self.connect_btn.setEnabled(True)

        except Exception as e:
            self.error_label.setText(f'Scan error: {str(e)}')
            self.status_label.setText('Device scan failed')
            # Add manual entry fallback
            self.device_combo.clear()
            self.device_combo.addItem('Manual: /dev/ttyUSB0', '/dev/ttyUSB0')
            self.device_combo.setEditable(True)

    def connect_device(self):
        """Connect to the temperature controller."""
        if not self.controller_factory:
            self.error_label.setText('Error: No controller factory configured')
            return

        # Get selected port from dropdown
        port = self.device_combo.currentData()

        if not port:
            self.error_label.setText('Error: No valid device selected')
            return

        try:
            # Use factory to create appropriate controller
            self.controller = self.controller_factory(port)
            self.is_connected = True

            # Update UI
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.device_combo.setEnabled(False)
            self.refresh_btn.setEnabled(
                False
            )  # Disable refresh while connected
            self.read_btn.setEnabled(True)
            self.error_report_btn.setEnabled(True)

            # Setup optional controls based on capabilities
            self.setup_optional_controls()

            # Read device info
            if isinstance(self.controller, DeviceIdentification):
                version = self.controller.get_firmware_version()
                device_id = self.controller.get_device_id()
                self.firmware_label.setText(version)
                self.device_id_label.setText(device_id)

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
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.device_combo.setEnabled(True)
        self.refresh_btn.setEnabled(True)  # Re-enable refresh
        self.read_btn.setEnabled(False)
        self.save_btn.setEnabled(False)
        self.error_report_btn.setEnabled(False)

        self.firmware_label.setText('--')
        self.device_id_label.setText('--')
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
            self.temp_setpoint.blockSignals(True)

            # Core temperature
            temp = self.controller.get_temperature_setpoint()
            self.temp_setpoint.setValue(temp)

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

            self.temp_setpoint.blockSignals(False)
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
        self.temp_value.setText(f'{actual_temp:.3f}')
        temp_dev = actual_temp - set_temp
        self.temp_dev_value.setText(f'{temp_dev:.3f}')

        # Update power/current display
        power = self.controller.get_output_power()
        self.power_value.setText(f'{power:.3f}')

        # Check if it's current (A) or percentage
        if isinstance(self.controller, CurrentLimitControl):
            self.power_unit.setText('A')
        else:
            self.power_unit.setText('%')

        voltage = self.controller.get_supply_voltage()
        self.voltage_value.setText(f'{voltage:.2f}')

        if self.controller.has_errors():
            error_code = self.controller.get_error_code()
            self.error_label.setText(f'Error code: {error_code}')
        else:
            self.error_label.setText('')

        # # Update stability status
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
        if self.show_temp_check.isChecked() and len(self.temp_data) > 0:
            ax1.plot(
                time_array,
                np.array(self.temp_data),
                color=plot_colors['temperature'],
                linewidth=2,
                label='Temperature',
            )

        # Plot deviation
        if self.show_dev_check.isChecked() and len(self.temp_dev_data) > 0:
            ax1.plot(
                time_array,
                np.array(self.temp_dev_data),
                color=plot_colors['temp_deviation'],
                linewidth=2,
                label='Deviation',
            )

        # Plot power/current
        if self.show_power_check.isChecked() and len(self.power_data) > 0:
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
        if isinstance(self.controller, CurrentLimitControl):
            ax2.set_ylabel('Current [A]')
        else:
            ax2.set_ylabel('Power [%]')

        ax1.grid(True, alpha=0.3)
        self.canvas.draw()

    # Parameter change handlers
    def on_temp_changed(self, value):
        """Handle temperature setpoint change."""
        if self.is_connected and self.controller:
            try:
                self.controller.set_temperature_setpoint(value)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def apply_temp_change(self):
        value = self.temp_setpoint.value()
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

        print('    pip install "xtalbake[gui] @ git+https://github.com/Peter-Barrow/xtalbake.git"')
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
