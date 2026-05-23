import os
os.environ["PYQTGRAPH_QT_LIB"] = "PyQt6"

import csv
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from math import isnan

import serial
import serial.tools.list_ports

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QComboBox,
    QLineEdit,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QMessageBox,
)

import pyqtgraph as pg


BAUD_RATE = 115200
MAX_POINTS = 2400


class ThermalFlowPidApp(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Thermal Flow PID Monitor - Fixed Reference Mode")
        self.resize(1300, 900)

        self.serial_port = None
        self.is_connected = False
        self.is_logging = False
        self.log_file = None
        self.csv_writer = None

        # Timing references
        self.first_data_millis = None
        self.log_start_millis = None

        # Data buffers
        self.time_s = deque(maxlen=MAX_POINTS)
        self.t_ref = deque(maxlen=MAX_POINTS)
        self.t_ref_used = deque(maxlen=MAX_POINTS)
        self.t_heater = deque(maxlen=MAX_POINTS)
        self.t_target = deque(maxlen=MAX_POINTS)
        self.error = deque(maxlen=MAX_POINTS)

        self.pwm = deque(maxlen=MAX_POINTS)
        self.pwm_avg_10s = deque(maxlen=MAX_POINTS)
        self.pwm_avg_30s = deque(maxlen=MAX_POINTS)

        self.p_term = deque(maxlen=MAX_POINTS)
        self.i_term = deque(maxlen=MAX_POINTS)
        self.d_term = deque(maxlen=MAX_POINTS)

        self.auto_mode = deque(maxlen=MAX_POINTS)
        self.ref_mode = deque(maxlen=MAX_POINTS)

        self._build_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(50)

        self.refresh_ports()

    # ------------------------------------------------------------
    # UI
    # ------------------------------------------------------------

    def _build_ui(self):
        main_layout = QVBoxLayout()

        # -------------------------
        # Connection controls
        # -------------------------
        connection_layout = QHBoxLayout()

        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Refresh Ports")
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")
        self.status_label = QLabel("Disconnected")

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.connect_serial)
        self.disconnect_button.clicked.connect(self.disconnect_serial)

        connection_layout.addWidget(QLabel("Port:"))
        connection_layout.addWidget(self.port_combo)
        connection_layout.addWidget(self.refresh_button)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.disconnect_button)
        connection_layout.addWidget(self.status_label)

        main_layout.addLayout(connection_layout)

        # -------------------------
        # PID controls
        # -------------------------
        pid_group = QGroupBox("PID / Heater Control")
        pid_layout = QGridLayout()

        self.kp_edit = QLineEdit("15.0")
        self.ki_edit = QLineEdit("0.5")
        self.kd_edit = QLineEdit("0.0")
        self.offset_edit = QLineEdit("5.0")
        self.pwm_max_edit = QLineEdit("80")
        self.manual_pwm_edit = QLineEdit("0")

        self.send_pid_button = QPushButton("Send PID Settings")
        self.auto_on_button = QPushButton("AUTO ON")
        self.auto_off_button = QPushButton("AUTO OFF")
        self.send_manual_pwm_button = QPushButton("Send Manual PWM")
        self.print_settings_button = QPushButton("Print Settings")

        self.send_pid_button.clicked.connect(self.send_pid_settings)
        self.auto_on_button.clicked.connect(lambda: self.send_command("AUTO 1"))
        self.auto_off_button.clicked.connect(lambda: self.send_command("AUTO 0"))
        self.send_manual_pwm_button.clicked.connect(self.send_manual_pwm)
        self.print_settings_button.clicked.connect(lambda: self.send_command("PRINT"))

        pid_layout.addWidget(QLabel("Kp"), 0, 0)
        pid_layout.addWidget(self.kp_edit, 0, 1)
        pid_layout.addWidget(QLabel("Ki"), 0, 2)
        pid_layout.addWidget(self.ki_edit, 0, 3)
        pid_layout.addWidget(QLabel("Kd"), 0, 4)
        pid_layout.addWidget(self.kd_edit, 0, 5)

        pid_layout.addWidget(QLabel("Target Offset °C"), 1, 0)
        pid_layout.addWidget(self.offset_edit, 1, 1)
        pid_layout.addWidget(QLabel("PWM Max %"), 1, 2)
        pid_layout.addWidget(self.pwm_max_edit, 1, 3)
        pid_layout.addWidget(QLabel("Manual PWM %"), 1, 4)
        pid_layout.addWidget(self.manual_pwm_edit, 1, 5)

        pid_layout.addWidget(self.send_pid_button, 2, 0, 1, 2)
        pid_layout.addWidget(self.auto_on_button, 2, 2)
        pid_layout.addWidget(self.auto_off_button, 2, 3)
        pid_layout.addWidget(self.send_manual_pwm_button, 2, 4)
        pid_layout.addWidget(self.print_settings_button, 2, 5)

        pid_group.setLayout(pid_layout)
        main_layout.addWidget(pid_group)

        # -------------------------
        # Reference controls
        # -------------------------
        ref_group = QGroupBox("Reference Mode")
        ref_layout = QHBoxLayout()

        self.ref_live_button = QPushButton("REFMODE LIVE")
        self.ref_fixed_button = QPushButton("REFMODE FIXED")
        self.capture_ref_button = QPushButton("SETREF / Capture Current Ref")

        self.ref_live_button.clicked.connect(lambda: self.send_command("REFMODE LIVE"))
        self.ref_fixed_button.clicked.connect(lambda: self.send_command("REFMODE FIXED"))
        self.capture_ref_button.clicked.connect(lambda: self.send_command("SETREF"))

        ref_layout.addWidget(self.ref_live_button)
        ref_layout.addWidget(self.ref_fixed_button)
        ref_layout.addWidget(self.capture_ref_button)

        ref_group.setLayout(ref_layout)
        main_layout.addWidget(ref_group)

        # -------------------------
        # Current values
        # -------------------------
        values_layout = QHBoxLayout()

        self.value_ref = QLabel("T_ref live: -- °C")
        self.value_ref_used = QLabel("T_ref used: -- °C")
        self.value_heater = QLabel("T_heater: -- °C")
        self.value_target = QLabel("Target: -- °C")
        self.value_error = QLabel("Error: -- °C")
        self.value_pwm = QLabel("PWM: -- %")
        self.value_pwm_avg = QLabel("PWM avg: -- / -- %")
        self.value_modes = QLabel("AUTO: -- | REF: --")

        for label in [
            self.value_ref,
            self.value_ref_used,
            self.value_heater,
            self.value_target,
            self.value_error,
            self.value_pwm,
            self.value_pwm_avg,
            self.value_modes,
        ]:
            label.setStyleSheet("font-size: 13px; font-weight: bold;")
            values_layout.addWidget(label)

        main_layout.addLayout(values_layout)

        # -------------------------
        # Plots
        # -------------------------
        pg.setConfigOptions(antialias=True)

        # Pens / colors
        pen_ref = pg.mkPen(color=(0, 170, 255), width=2)          # blue
        pen_ref_used = pg.mkPen(color=(120, 220, 255), width=2)   # light blue
        pen_heater = pg.mkPen(color=(255, 120, 0), width=2)       # orange
        pen_target = pg.mkPen(color=(0, 220, 80), width=2)        # green

        pen_pwm = pg.mkPen(color=(180, 80, 255), width=1)         # purple
        pen_pwm_avg_10s = pg.mkPen(color=(255, 200, 0), width=2)  # yellow
        pen_pwm_avg_30s = pg.mkPen(color=(0, 220, 180), width=2)  # teal
        pen_error = pg.mkPen(color=(255, 60, 60), width=2)        # red

        pen_p = pg.mkPen(color=(255, 200, 0), width=2)            # yellow
        pen_i = pg.mkPen(color=(0, 220, 180), width=2)            # teal
        pen_d = pg.mkPen(color=(255, 80, 180), width=2)           # pink

        # Temperature plot
        self.temp_plot = pg.PlotWidget(title="Temperatures")
        self.temp_plot.setLabel("left", "Temperature", units="°C")
        self.temp_plot.setLabel("bottom", "Session time", units="s")
        self.temp_plot.addLegend()
        self.temp_plot.showGrid(x=True, y=True)

        self.curve_ref = self.temp_plot.plot(pen=pen_ref, name="T_ref live")
        self.curve_ref_used = self.temp_plot.plot(pen=pen_ref_used, name="T_ref used")
        self.curve_heater = self.temp_plot.plot(pen=pen_heater, name="T_heater")
        self.curve_target = self.temp_plot.plot(pen=pen_target, name="Target")

        # Control plot
        self.control_plot = pg.PlotWidget(title="PWM Output, Moving Averages, and Error")
        self.control_plot.setLabel("left", "PWM % / Error °C")
        self.control_plot.setLabel("bottom", "Session time", units="s")
        self.control_plot.addLegend()
        self.control_plot.showGrid(x=True, y=True)

        self.curve_pwm = self.control_plot.plot(
            pen=pen_pwm,
            name="PWM raw %",
        )

        self.curve_pwm_avg_10s = self.control_plot.plot(
            pen=pen_pwm_avg_10s,
            name="PWM avg 10 s",
        )

        self.curve_pwm_avg_30s = self.control_plot.plot(
            pen=pen_pwm_avg_30s,
            name="PWM avg 30 s",
        )

        self.curve_error = self.control_plot.plot(
            pen=pen_error,
            name="Error °C",
        )

        # PID plot
        self.pid_plot = pg.PlotWidget(title="PID Terms")
        self.pid_plot.setLabel("left", "PID contribution")
        self.pid_plot.setLabel("bottom", "Session time", units="s")
        self.pid_plot.addLegend()
        self.pid_plot.showGrid(x=True, y=True)

        self.curve_p = self.pid_plot.plot(pen=pen_p, name="P term")
        self.curve_i = self.pid_plot.plot(pen=pen_i, name="I term")
        self.curve_d = self.pid_plot.plot(pen=pen_d, name="D term")

        main_layout.addWidget(self.temp_plot)
        main_layout.addWidget(self.control_plot)
        main_layout.addWidget(self.pid_plot)

        # -------------------------
        # Logging
        # -------------------------
        logging_layout = QHBoxLayout()

        self.start_log_button = QPushButton("Start CSV Log")
        self.stop_log_button = QPushButton("Stop Log")
        self.clear_button = QPushButton("Clear Plots")

        self.start_log_button.clicked.connect(self.start_logging)
        self.stop_log_button.clicked.connect(self.stop_logging)
        self.clear_button.clicked.connect(self.clear_data)

        self.log_label = QLabel("Logging: OFF")

        logging_layout.addWidget(self.start_log_button)
        logging_layout.addWidget(self.stop_log_button)
        logging_layout.addWidget(self.clear_button)
        logging_layout.addWidget(self.log_label)

        main_layout.addLayout(logging_layout)

        self.setLayout(main_layout)

    # ------------------------------------------------------------
    # Serial
    # ------------------------------------------------------------

    def refresh_ports(self):
        self.port_combo.clear()

        ports = serial.tools.list_ports.comports()

        for port in ports:
            self.port_combo.addItem(
                f"{port.device} - {port.description}",
                port.device,
            )

    def connect_serial(self):
        if self.is_connected:
            return

        port = self.port_combo.currentData()

        if not port:
            QMessageBox.warning(self, "No port selected", "Please select a serial port.")
            return

        try:
            self.serial_port = serial.Serial(port, BAUD_RATE, timeout=0.05)
            time.sleep(1.0)
            self.serial_port.reset_input_buffer()

            self.is_connected = True
            self.status_label.setText(f"Connected to {port}")

        except Exception as exc:
            QMessageBox.critical(self, "Connection error", str(exc))

    def disconnect_serial(self):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass

        self.serial_port = None
        self.is_connected = False
        self.status_label.setText("Disconnected")

    def send_command(self, command: str):
        if not self.is_connected or self.serial_port is None:
            QMessageBox.warning(self, "Not connected", "Connect to the Arduino first.")
            return

        try:
            self.serial_port.write((command.strip() + "\n").encode("utf-8"))
        except Exception as exc:
            QMessageBox.critical(self, "Serial write error", str(exc))

    def send_pid_settings(self):
        commands = [
            f"KP {self.kp_edit.text()}",
            f"KI {self.ki_edit.text()}",
            f"KD {self.kd_edit.text()}",
            f"OFFSET {self.offset_edit.text()}",
            f"PWM_MAX {self.pwm_max_edit.text()}",
        ]

        for command in commands:
            self.send_command(command)
            time.sleep(0.05)

    def send_manual_pwm(self):
        self.send_command(f"PWM {self.manual_pwm_edit.text()}")

    # ------------------------------------------------------------
    # Data helpers
    # ------------------------------------------------------------

    @staticmethod
    def compute_time_window_average(time_values, data_values, window_s):
        """
        Compute average of data_values over the last window_s seconds.
        """

        if len(time_values) == 0 or len(data_values) == 0:
            return float("nan")

        current_time = time_values[-1]
        cutoff_time = current_time - window_s

        values = [
            value
            for t, value in zip(time_values, data_values)
            if t >= cutoff_time
        ]

        if len(values) == 0:
            return float("nan")

        return sum(values) / len(values)

    # ------------------------------------------------------------
    # Data handling
    # ------------------------------------------------------------

    def update_loop(self):
        if not self.is_connected or self.serial_port is None:
            return

        try:
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode(
                    "utf-8",
                    errors="ignore",
                ).strip()
                self.process_line(line)

        except Exception as exc:
            self.status_label.setText(f"Serial error: {exc}")

        self.update_plots()

    def process_line(self, line: str):
        if not line:
            return

        if line.startswith("#"):
            print(line)
            return

        if line.startswith("millis"):
            print(line)
            return

        parts = line.split(",")

        if len(parts) != 12:
            return

        try:
            millis = float(parts[0])
            t_ref = float(parts[1])
            t_ref_used = float(parts[2])
            t_heater = float(parts[3])
            t_target = float(parts[4])
            error = float(parts[5])
            pwm = float(parts[6])
            p_term = float(parts[7])
            i_term = float(parts[8])
            d_term = float(parts[9])
            auto_mode = int(float(parts[10]))
            ref_mode = int(float(parts[11]))

        except ValueError:
            return

        # Session timing starts at first received data point.
        if self.first_data_millis is None:
            self.first_data_millis = millis

        # Log timing starts at first received data point after Start CSV Log.
        if self.is_logging and self.log_start_millis is None:
            self.log_start_millis = millis

        session_time_s = (millis - self.first_data_millis) / 1000.0

        if self.log_start_millis is not None:
            log_time_s = (millis - self.log_start_millis) / 1000.0
        else:
            log_time_s = float("nan")

        # Append main data
        self.time_s.append(session_time_s)
        self.t_ref.append(t_ref)
        self.t_ref_used.append(t_ref_used)
        self.t_heater.append(t_heater)
        self.t_target.append(t_target)
        self.error.append(error)
        self.pwm.append(pwm)
        self.p_term.append(p_term)
        self.i_term.append(i_term)
        self.d_term.append(d_term)
        self.auto_mode.append(auto_mode)
        self.ref_mode.append(ref_mode)

        # Compute moving averages after appending current point
        pwm_avg_10s = self.compute_time_window_average(
            self.time_s,
            self.pwm,
            10.0,
        )

        pwm_avg_30s = self.compute_time_window_average(
            self.time_s,
            self.pwm,
            30.0,
        )

        self.pwm_avg_10s.append(pwm_avg_10s)
        self.pwm_avg_30s.append(pwm_avg_30s)

        ref_mode_text = "FIXED" if ref_mode == 1 else "LIVE"
        auto_text = "ON" if auto_mode == 1 else "OFF"

        self.value_ref.setText(f"T_ref live: {t_ref:.2f} °C")
        self.value_ref_used.setText(f"T_ref used: {t_ref_used:.2f} °C")
        self.value_heater.setText(f"T_heater: {t_heater:.2f} °C")
        self.value_target.setText(f"Target: {t_target:.2f} °C")
        self.value_error.setText(f"Error: {error:.2f} °C")
        self.value_pwm.setText(f"PWM: {pwm:.1f} %")

        if isnan(pwm_avg_10s):
            avg10_text = "--"
        else:
            avg10_text = f"{pwm_avg_10s:.1f}"

        if isnan(pwm_avg_30s):
            avg30_text = "--"
        else:
            avg30_text = f"{pwm_avg_30s:.1f}"

        self.value_pwm_avg.setText(f"PWM avg 10/30s: {avg10_text} / {avg30_text} %")
        self.value_modes.setText(f"AUTO: {auto_text} | REF: {ref_mode_text}")

        if self.is_logging and self.csv_writer is not None:
            self.csv_writer.writerow([
                datetime.now().isoformat(),
                millis,
                session_time_s,
                log_time_s,
                t_ref,
                t_ref_used,
                t_heater,
                t_target,
                error,
                pwm,
                pwm_avg_10s,
                pwm_avg_30s,
                p_term,
                i_term,
                d_term,
                auto_mode,
                ref_mode,
            ])

    def update_plots(self):
        if len(self.time_s) < 2:
            return

        x = list(self.time_s)

        self.curve_ref.setData(x, list(self.t_ref))
        self.curve_ref_used.setData(x, list(self.t_ref_used))
        self.curve_heater.setData(x, list(self.t_heater))
        self.curve_target.setData(x, list(self.t_target))

        self.curve_pwm.setData(x, list(self.pwm))
        self.curve_pwm_avg_10s.setData(x, list(self.pwm_avg_10s))
        self.curve_pwm_avg_30s.setData(x, list(self.pwm_avg_30s))
        self.curve_error.setData(x, list(self.error))

        self.curve_p.setData(x, list(self.p_term))
        self.curve_i.setData(x, list(self.i_term))
        self.curve_d.setData(x, list(self.d_term))

    def clear_data(self):
        self.time_s.clear()
        self.t_ref.clear()
        self.t_ref_used.clear()
        self.t_heater.clear()
        self.t_target.clear()
        self.error.clear()

        self.pwm.clear()
        self.pwm_avg_10s.clear()
        self.pwm_avg_30s.clear()

        self.p_term.clear()
        self.i_term.clear()
        self.d_term.clear()

        self.auto_mode.clear()
        self.ref_mode.clear()

        self.first_data_millis = None

    # ------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------

    def start_logging(self):
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save CSV Log",
            f"thermal_flow_pid_fixed_ref_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv)",
        )

        if not path:
            return

        self.log_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow([
            "timestamp_iso",
            "millis",
            "session_time_s",
            "log_time_s",
            "T_ref_live_C",
            "T_ref_used_C",
            "T_heater_C",
            "T_target_C",
            "error_C",
            "pwm_percent",
            "pwm_avg_10s",
            "pwm_avg_30s",
            "p_term",
            "i_term",
            "d_term",
            "auto_mode",
            "ref_mode",
        ])

        # This will be set on the next parsed serial data line.
        self.log_start_millis = None

        self.is_logging = True
        self.log_label.setText(f"Logging: {Path(path).name}")

    def stop_logging(self):
        if self.log_file is not None:
            self.log_file.close()

        self.log_file = None
        self.csv_writer = None
        self.is_logging = False
        self.log_start_millis = None
        self.log_label.setText("Logging: OFF")

    def closeEvent(self, event):
        self.stop_logging()
        self.disconnect_serial()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ThermalFlowPidApp()
    window.show()
    sys.exit(app.exec())