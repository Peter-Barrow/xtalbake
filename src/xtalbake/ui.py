"""
MTD1020T GUI Application using PyQt6 and Matplotlib
"""

import sys
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QDoubleSpinBox,
    QGroupBox,
    QStatusBar,
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import numpy as np
from collections import deque
from datetime import datetime

from xtalbake import MTD1020T

matplotlib.use('Qt5Agg')

plot_colors = {
    'temperature': '#E76F51',
    'temp_deviation': '#F4A261',
    'current': '#2A9D8F',
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
        # self.fig.tight_layout()


class MTD1020TGUI(QMainWindow):
    """Main GUI window for MTD1020T Temperature Controller."""

    def __init__(self):
        super().__init__()
        self.controller = None
        self.is_connected = False

        # Data storage for plotting (30 seconds at ~0.5s update rate)
        self.max_points = 60
        self.time_data = deque(maxlen=self.max_points)
        self.temp_data = deque(maxlen=self.max_points)
        self.temp_dev_data = deque(maxlen=self.max_points)
        self.current_data = deque(maxlen=self.max_points)
        self.start_time = None

        self.init_ui()

        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_values)

    def init_ui(self):
        self.setWindowTitle('xtalbake MTD1020T GUI')
        self.resize(900, 850)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        left_column = QVBoxLayout()

        self.chart_widget = self.create_chart_group()
        left_column.addWidget(self.chart_widget, stretch=3)

        control_layout = QVBoxLayout()
        control_layout.setSpacing(10)

        control_layout.addWidget(self.create_parameters_group(), stretch=1)

        left_column.addLayout(control_layout, stretch=1)

        right_column = QVBoxLayout()
        # right_column.setSpacing(10)
        right_column.setAlignment(Qt.AlignmentFlag.AlignTop)

        right_column.addWidget(self.create_readouts_group(), stretch=3)
        right_column.addWidget(self.create_connection_group(), stretch=1)
        right_column.addWidget(self.create_device_info_group(), stretch=1)
        right_column.addWidget(self.create_buttons_group(), stretch=1)

        main_layout.addLayout(left_column, stretch=3)
        main_layout.addLayout(right_column, stretch=1)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel('Disconnected')
        self.error_label = QLabel('')
        self.error_label.setStyleSheet('color: red; font-weight: bold;')
        self.time_label = QLabel('')
        self.enabled_label = QLabel('Disabled')
        self.status_bar.addWidget(self.status_label)
        self.status_bar.addWidget(self.enabled_label, 1)
        self.status_bar.addWidget(self.error_label, 1)
        self.status_bar.addPermanentWidget(self.time_label)

    def create_connection_group(self):
        """Create connection control group."""
        group = QGroupBox()
        layout = QGridLayout()

        layout.addWidget(QLabel('Serial Port:'), 0, 0)
        self.port_input = QLineEdit('/dev/ttyUSB0')
        layout.addWidget(self.port_input, 0, 1)

        self.connect_btn = QPushButton('Connect')
        self.connect_btn.clicked.connect(self.connect_device)
        layout.addWidget(self.connect_btn, 1, 0)

        self.disconnect_btn = QPushButton('Disconnect')
        self.disconnect_btn.clicked.connect(self.disconnect_device)
        self.disconnect_btn.setEnabled(False)
        layout.addWidget(self.disconnect_btn, 1, 1)

        group.setLayout(layout)
        return group

    def create_device_info_group(self):
        """Create device information group."""
        group = QGroupBox()
        layout = QGridLayout()

        layout.addWidget(QLabel('Module Name:'), 0, 0)
        self.module_label = QLabel('MTD1020T')
        layout.addWidget(self.module_label, 0, 1)

        layout.addWidget(QLabel('Firmware Version:'), 1, 0)
        self.firmware_label = QLabel('--')
        layout.addWidget(self.firmware_label, 1, 1)

        layout.addWidget(QLabel('Temperature Range:'), 2, 0)
        self.temp_range_label = QLabel('5°C to 45°C')
        layout.addWidget(self.temp_range_label, 2, 1)

        layout.addWidget(QLabel('UUID:'), 3, 0)
        self.uuid_label = QLabel('--')
        self.uuid_label.setFont(QFont('Courier', 8))
        layout.addWidget(self.uuid_label, 3, 1)

        group.setLayout(layout)
        return group

    def create_parameters_group(self):
        """Create MTD program parameters group."""
        group = QGroupBox()
        layout = QGridLayout()

        row = 0

        # Set Temperature
        layout.addWidget(QLabel('Set Temperature:'), row, 0)
        self.temp_setpoint = QDoubleSpinBox()
        self.temp_setpoint.setRange(5.0, 45.0)
        self.temp_setpoint.setValue(27.0)
        self.temp_setpoint.setSuffix(' °C')
        self.temp_setpoint.valueChanged.connect(self.on_temp_changed)
        layout.addWidget(self.temp_setpoint, row, 1)
        row += 1

        # Temperature Window
        layout.addWidget(QLabel('Temperature Window:'), row, 0)
        self.temp_window = QSpinBox()
        self.temp_window.setRange(1, 32000)
        self.temp_window.setValue(1000)
        self.temp_window.setPrefix('± ')
        self.temp_window.setSuffix(' mK')
        self.temp_window.valueChanged.connect(self.on_window_changed)
        layout.addWidget(self.temp_window, row, 1)
        row += 1

        # P Share
        layout.addWidget(QLabel('P Share:'), row, 0)
        self.p_share = QDoubleSpinBox()
        self.p_share.setRange(0, 100000)
        self.p_share.setValue(1.0)
        self.p_share.setSuffix(' A/K')
        self.p_share.valueChanged.connect(self.on_p_changed)
        layout.addWidget(self.p_share, row, 1)
        row += 1

        # I Share
        layout.addWidget(QLabel('I Share:'), row, 0)
        self.i_share = QDoubleSpinBox()
        self.i_share.setRange(0, 100000)
        self.i_share.setValue(0.2)
        self.i_share.setSuffix(' A/(K*sec)')
        self.i_share.valueChanged.connect(self.on_i_changed)
        layout.addWidget(self.i_share, row, 1)
        row += 1

        # D Share
        layout.addWidget(QLabel('D Share:'), row, 0)
        self.d_share = QDoubleSpinBox()
        self.d_share.setRange(0, 100000)
        self.d_share.setValue(0.1)
        self.d_share.setSuffix(' (A*sec)/K')
        self.d_share.valueChanged.connect(self.on_d_changed)
        layout.addWidget(self.d_share, row, 1)
        row += 1

        # Cycle Time
        layout.addWidget(QLabel('Cycle Time:'), row, 0)
        self.cycle_time = QSpinBox()
        self.cycle_time.setRange(1, 1000)
        self.cycle_time.setValue(15)
        self.cycle_time.setSuffix(' ms')
        self.cycle_time.valueChanged.connect(self.on_cycle_changed)
        layout.addWidget(self.cycle_time, row, 1)
        row += 1

        # Current Limit
        layout.addWidget(QLabel('Current Limit:'), row, 0)
        self.current_limit = QDoubleSpinBox()
        self.current_limit.setRange(0.2, 2.0)
        self.current_limit.setValue(2.0)
        self.current_limit.setSuffix(' A')
        self.current_limit.valueChanged.connect(self.on_current_limit_changed)
        layout.addWidget(self.current_limit, row, 1)
        row += 1

        # Temperature Protection Delay
        layout.addWidget(QLabel('Temperature Protection Delay:'), row, 0)
        self.temp_delay = QSpinBox()
        self.temp_delay.setRange(1, 32000)
        self.temp_delay.setValue(10)
        self.temp_delay.setSuffix(' s')
        self.temp_delay.valueChanged.connect(self.on_delay_changed)
        layout.addWidget(self.temp_delay, row, 1)
        row += 1

        # Critical Gain
        layout.addWidget(QLabel('Critical Gain:'), row, 0)
        self.critical_gain = QSpinBox()
        self.critical_gain.setRange(10, 100000)
        self.critical_gain.setValue(2000)
        self.critical_gain.setSuffix(' mA/K')
        self.critical_gain.valueChanged.connect(self.on_critical_gain_changed)
        layout.addWidget(self.critical_gain, row, 1)
        row += 1

        # Critical Period
        layout.addWidget(QLabel('Critical Period Duration:'), row, 0)
        self.critical_period = QSpinBox()
        self.critical_period.setRange(100, 100000)
        self.critical_period.setValue(4000)
        self.critical_period.setSuffix(' ms')
        self.critical_period.valueChanged.connect(
            self.on_critical_period_changed
        )
        layout.addWidget(self.critical_period, row, 1)

        group.setLayout(layout)
        return group

    def create_buttons_group(self):
        """Create functional buttons group."""
        group = QGroupBox()

        layout = QHBoxLayout()

        left_column = QVBoxLayout()
        save_flash_btn = QPushButton('Save Settings in MTD Flash')
        save_flash_btn.clicked.connect(self.save_to_flash)
        left_column.addWidget(save_flash_btn)
        save_file_btn = QPushButton('Save Settings to File')
        left_column.addWidget(save_file_btn)
        layout.addLayout(left_column)

        right_column = QVBoxLayout()
        read_btn = QPushButton('Read Settings from MTD')
        read_btn.clicked.connect(self.read_from_device)
        right_column.addWidget(read_btn)
        load_file_btn = QPushButton('Load Settings from File')
        right_column.addWidget(load_file_btn)
        layout.addLayout(right_column)

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
                /* keep native look */
                border: 1px solid palette(mid);
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QPushButton:checked {{
                border: 2px solid {plot_colors['temperature']};
            }}
        """)

        self.show_dev_check = QPushButton('Temperature Deviation')
        self.show_dev_check.setCheckable(True)
        self.show_dev_check.setChecked(True)
        self.show_dev_check.clicked.connect(self.update_plot)
        self.show_dev_check.setStyleSheet(f"""
            QPushButton {{
                /* keep native look */
                border: 1px solid palette(mid);
                border-radius: 4px;
                padding: 4px 8px;
            }}
            QPushButton:checked {{
                border: 2px solid {plot_colors['temp_deviation']};
            }}
        """)

        self.show_current_check = QPushButton('Current')
        self.show_current_check.setCheckable(True)
        self.show_current_check.setChecked(True)
        self.show_current_check.clicked.connect(self.update_plot)
        self.show_current_check.setStyleSheet(f"""
            QPushButton {{
                /* keep native look */
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
        legend_layout.addWidget(self.show_current_check)
        legend_layout.addStretch()

        layout.addLayout(legend_layout)

        group.setLayout(layout)
        return group

    def create_readouts_group(self):
        """Create actual read-out values group."""
        group = QGroupBox()
        layout = QGridLayout()

        # Create large display labels
        font_large = QFont()
        font_large.setPointSize(24)
        font_large.setBold(True)

        # # Status
        # layout.addWidget(QLabel('Status'), 0, 0)
        # self.status_value = QLabel('Disabled')
        # self.status_value.setFont(font_large)
        # self.status_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # layout.addWidget(self.status_value, 1, 0)

        # Current
        layout.addWidget(QLabel('Current'), 0, 0)
        current_widget = QWidget()
        current_layout = QHBoxLayout(current_widget)
        self.current_value = QLabel('--')
        self.current_value.setFont(font_large)
        self.current_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        current_layout.addWidget(self.current_value)
        current_layout.addWidget(QLabel('A'))
        layout.addWidget(current_widget, 1, 0)

        # Voltage
        layout.addWidget(QLabel('Voltage'), 2, 0)
        voltage_widget = QWidget()
        voltage_layout = QHBoxLayout(voltage_widget)
        self.voltage_value = QLabel('--')
        self.voltage_value.setFont(font_large)
        self.voltage_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        voltage_layout.addWidget(self.voltage_value)
        voltage_layout.addWidget(QLabel('V'))
        layout.addWidget(voltage_widget, 3, 0)

        # Temperature
        layout.addWidget(QLabel('Temperature'), 0, 1)
        temp_widget = QWidget()
        temp_layout = QHBoxLayout(temp_widget)
        self.temp_value = QLabel('--')
        self.temp_value.setFont(font_large)
        self.temp_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        temp_layout.addWidget(self.temp_value)
        temp_layout.addWidget(QLabel('°C'))
        layout.addWidget(temp_widget, 1, 1)

        # Temperature Deviation
        layout.addWidget(QLabel('Temperature Deviation'), 2, 1)
        dev_widget = QWidget()
        # dev_widget.setStyleSheet('background-color: red;')
        dev_layout = QHBoxLayout(dev_widget)
        self.temp_dev_value = QLabel('--')
        self.temp_dev_value.setFont(font_large)
        self.temp_dev_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # self.temp_dev_value.setStyleSheet('color: white;')
        dev_layout.addWidget(self.temp_dev_value)
        self.temp_dev_unit = QLabel('mK')
        # self.temp_dev_unit.setStyleSheet('color: white;')
        dev_layout.addWidget(self.temp_dev_unit)
        layout.addWidget(dev_widget, 3, 1)

        group.setLayout(layout)
        return group

    def connect_device(self):
        """Connect to the MTD1020T device."""
        port = self.port_input.text()
        try:
            self.controller = MTD1020T(port=port)
            self.is_connected = True

            # Update UI
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.port_input.setEnabled(False)

            # Read device info
            version = self.controller.version
            uuid = self.controller.uuid
            self.firmware_label.setText(version)
            self.uuid_label.setText(uuid)

            # Read current settings
            self.read_from_device()

            # Start update timer
            self.start_time = datetime.now()
            self.update_timer.start(250)  # Update every 500ms

            self.status_label.setText(f'Connected to {port}')
            self.error_label.setText('')

        except Exception as e:
            self.status_label.setText('Connection Failed')
            self.error_label.setText(f'Error: {str(e)}')

    def disconnect_device(self):
        """Disconnect from the device."""
        if self.controller:
            self.update_timer.stop()
            self.controller._close()
            self.controller = None
            self.is_connected = False

            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.port_input.setEnabled(True)

            self.status_label.setText('Disconnected')
            self.firmware_label.setText('--')
            self.uuid_label.setText('--')

    def read_from_device(self):
        """Read current settings from device."""
        if not self.is_connected or not self.controller:
            return

        try:
            # Block signals to prevent writing back
            self.temp_setpoint.blockSignals(True)
            self.temp_window.blockSignals(True)
            self.p_share.blockSignals(True)
            self.i_share.blockSignals(True)
            self.d_share.blockSignals(True)
            self.cycle_time.blockSignals(True)
            self.current_limit.blockSignals(True)
            self.temp_delay.blockSignals(True)
            self.critical_gain.blockSignals(True)
            self.critical_period.blockSignals(True)

            # Read values
            self.temp_setpoint.setValue(
                self.controller.temperature_set_point / 1000.0
            )
            self.temp_window.setValue(
                self.controller.temperature_set_point_window
            )
            self.p_share.setValue(self.controller.pid_p_share / 1000.0)
            self.i_share.setValue(self.controller.pid_i_share / 1000.0)
            self.d_share.setValue(self.controller.pid_d_share / 1000.0)
            self.cycle_time.setValue(self.controller.pid_cycle_time)
            self.current_limit.setValue(self.controller.current_limit / 1000.0)
            self.temp_delay.setValue(self.controller.temperature_control_delay)
            self.critical_gain.setValue(self.controller.loop_test_critical_gain)
            self.critical_period.setValue(
                self.controller.loop_test_critical_period
            )

            # Unblock signals
            self.temp_setpoint.blockSignals(False)
            self.temp_window.blockSignals(False)
            self.p_share.blockSignals(False)
            self.i_share.blockSignals(False)
            self.d_share.blockSignals(False)
            self.cycle_time.blockSignals(False)
            self.current_limit.blockSignals(False)
            self.temp_delay.blockSignals(False)
            self.critical_gain.blockSignals(False)
            self.critical_period.blockSignals(False)

            self.status_label.setText('Settings read from device')

        except Exception as e:
            self.error_label.setText(f'Read error: {str(e)}')

    def save_to_flash(self):
        """Save current settings to device flash memory."""
        if not self.is_connected or not self.controller:
            return

        try:
            self.controller.save_to_memory()
            self.status_label.setText('Settings saved to device flash')
        except Exception as e:
            self.error_label.setText(f'Save error: {str(e)}')

    def update_values(self):
        """Update displayed values from device."""
        if not self.is_connected or not self.controller:
            return

        try:
            # Read current values
            actual_temp = self.controller.actual_temperature
            set_temp = self.controller.temperature_set_point
            current, mode = self.controller.actual_current
            voltage = self.controller.actual_voltage

            # Update displays
            self.temp_value.setText(f'{actual_temp / 1000.0:.3f}')
            temp_dev = actual_temp - set_temp
            self.temp_dev_value.setText(f'{temp_dev:.0f}')
            self.current_value.setText(f'{current / 1000.0:.2f}')
            self.voltage_value.setText(f'{voltage / 1000.0:.2f}')

            # Check errors
            errors = self.controller.errors
            active_errors = [name for name, active in errors.items() if active]
            if active_errors:
                self.error_label.setText(f'Errors: {", ".join(active_errors)}')
            else:
                self.error_label.setText('')

            # Update time
            self.time_label.setText(
                datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            )

            # Add to plot data
            if self.start_time:
                elapsed = (datetime.now() - self.start_time).total_seconds()
                self.time_data.append(elapsed)
                self.temp_data.append(actual_temp / 1000.0)
                self.temp_dev_data.append(temp_dev / 1000.0)
                self.current_data.append(current / 1000.0)

                self.update_plot()

        except Exception as e:
            self.error_label.setText(f'Update error: {str(e)}')

    def update_plot(self):
        if len(self.time_data) == 0:
            return

        # Clear the figure and recreate axes to avoid accumulation
        self.canvas.fig.clear()
        ax1 = self.canvas.fig.add_subplot(111)

        ax1.set_facecolor('none')

        ax1.spines['top'].set_visible(False)
        ax1.spines['right'].set_visible(False)

        # Setup twin axes for current
        ax2 = ax1.twinx()

        ax2.spines['top'].set_visible(False)

        time_array = np.array(self.time_data)

        lines = []
        labels = []

        if self.show_temp_check.isChecked() and len(self.temp_data) > 0:
            (line,) = ax1.plot(
                time_array,
                np.array(self.temp_data),
                plot_colors['temperature'],
                linewidth=2,
                label='Temperature',
            )
            lines.append(line)
            labels.append('Temperature')

        if self.show_dev_check.isChecked() and len(self.temp_dev_data) > 0:
            (line,) = ax1.plot(
                time_array,
                np.array(self.temp_dev_data),
                plot_colors['temp_deviation'],
                linewidth=2,
                label='Temperature Deviation',
            )
            lines.append(line)
            labels.append('Temperature Deviation')

        if self.show_current_check.isChecked() and len(self.current_data) > 0:
            (line,) = ax2.plot(
                time_array,
                np.array(self.current_data),
                plot_colors['current'],
                linewidth=2,
                label='Current',
            )
            lines.append(line)
            labels.append('Current')

        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Temperature [°C] / Deviation [K]')
        ax2.set_ylabel('Current [A]')

        ax1.grid(True, alpha=0.3)

        self.canvas.draw()

    # Parameter change handlers
    def on_temp_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.temperature_set_point = int(value * 1000)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_window_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.temperature_set_point_window = value
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_p_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.pid_p_share = int(value * 1000)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_i_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.pid_i_share = int(value * 1000)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_d_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.pid_d_share = int(value * 1000)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_cycle_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.pid_cycle_time = value
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_current_limit_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.current_limit = int(value * 1000)
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_delay_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.temperature_control_delay = value
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_critical_gain_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.loop_test_critical_gain = value
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')

    def on_critical_period_changed(self, value):
        if self.is_connected and self.controller:
            try:
                self.controller.loop_test_critical_period = value
            except Exception as e:
                self.error_label.setText(f'Error: {str(e)}')


def main():
    app = QApplication(sys.argv)
    window = MTD1020TGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
