import sys
import csv
from collections import deque
from datetime import datetime

from PyQt6.QtCore import QThread
from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QComboBox,
    QGridLayout,
)

import pyqtgraph as pg

from serial_worker import SerialWorker


class ThermalFlowApp(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Thermal Flow Test App")
        self.resize(1200, 800)

        self.max_points = 500

        self.time_data = deque(maxlen=self.max_points)
        self.t_up_data = deque(maxlen=self.max_points)
        self.t_down_data = deque(maxlen=self.max_points)
        self.t_heater_data = deque(maxlen=self.max_points)
        self.delta_data = deque(maxlen=self.max_points)
        self.delta_corr_data = deque(maxlen=self.max_points)
        self.duty_data = deque(maxlen=self.max_points)

        self.worker = None
        self.thread = None

        self.build_ui()
        self.refresh_ports()
        self.csv_buffer = []

    def build_ui(self):
        main_layout = QHBoxLayout(self)

        control_layout = QVBoxLayout()

        self.port_combo = QComboBox()
        self.refresh_button = QPushButton("Refresh Ports")
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")
        self.save_button = QPushButton("Save CSV")
        self.status_label = QLabel("Status: Disconnected")

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.connect_serial)
        self.disconnect_button.clicked.connect(self.disconnect_serial)
        self.save_button.clicked.connect(self.save_csv)

        control_layout.addWidget(self.save_button)

        control_layout.addWidget(QLabel("Serial Port"))
        control_layout.addWidget(self.port_combo)
        control_layout.addWidget(self.refresh_button)
        control_layout.addWidget(self.connect_button)
        control_layout.addWidget(self.disconnect_button)
        control_layout.addWidget(self.status_label)
        control_layout.addStretch()

        main_layout.addLayout(control_layout, 1)

        plot_layout = QGridLayout()

        self.temp_plot = pg.PlotWidget(title="Temperatures")
        self.temp_plot.setLabel("left", "Temperature", units="°C")
        self.temp_plot.setLabel("bottom", "Time", units="s")
        self.temp_plot.addLegend()

        self.delta_plot = pg.PlotWidget(title="Delta T")
        self.delta_plot.setLabel("left", "ΔT", units="°C")
        self.delta_plot.setLabel("bottom", "Time", units="s")
        self.delta_plot.addLegend()

        self.duty_plot = pg.PlotWidget(title="Heater Duty")
        self.duty_plot.setLabel("left", "Duty", units="/255")
        self.duty_plot.setLabel("bottom", "Time", units="s")

        self.flow_plot = pg.PlotWidget(title="Future Flow Estimate")
        self.flow_plot.setLabel("left", "Flow", units="SLPM")
        self.flow_plot.setLabel("bottom", "Time", units="s")

        self.t_up_curve = self.temp_plot.plot(
            name="T_up", pen=pg.mkPen("b", width=2)
        )
        self.t_down_curve = self.temp_plot.plot(
            name="T_down", pen=pg.mkPen("r", width=2)
        )
        self.t_heater_curve = self.temp_plot.plot(
            name="T_heater", pen=pg.mkPen("y", width=2)
        )

        self.delta_curve = self.delta_plot.plot(
            name="deltaT", pen=pg.mkPen("g", width=2)
        )
        self.delta_corr_curve = self.delta_plot.plot(
            name="deltaT_corr", pen=pg.mkPen("m", width=2)
        )

        self.duty_curve = self.duty_plot.plot(
            pen=pg.mkPen("c", width=2)
        )

        plot_layout.addWidget(self.temp_plot, 0, 0)
        plot_layout.addWidget(self.delta_plot, 0, 1)
        plot_layout.addWidget(self.duty_plot, 1, 0)
        plot_layout.addWidget(self.flow_plot, 1, 1)

        main_layout.addLayout(plot_layout, 4)

    def refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItems(SerialWorker.available_ports())

    def connect_serial(self):
        port = self.port_combo.currentText()
        if not port:
            self.status_label.setText("Status: No serial port selected")
            return

        self.thread = QThread()
        self.worker = SerialWorker(port_name=port, baudrate=115200)
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.sample_received.connect(self.handle_sample)
        self.worker.status_changed.connect(self.update_status)
        self.worker.raw_line_received.connect(lambda line: print("RAW:", line))

        self.thread.start()

    def disconnect_serial(self):
        if self.worker:
            self.worker.stop()

        if self.thread:
            self.thread.quit()
            self.thread.wait()

        self.worker = None
        self.thread = None

    def update_status(self, message: str):
        self.status_label.setText(f"Status: {message}")

    def handle_sample(self, sample):
        # print("SAMPLE:", sample)
        t = sample.time_ms / 1000.0

        self.time_data.append(t)
        self.t_up_data.append(sample.t_up)
        self.t_down_data.append(sample.t_down)
        self.t_heater_data.append(sample.t_heater)
        self.delta_data.append(sample.delta_t)
        self.delta_corr_data.append(sample.delta_t_corr)
        self.duty_data.append(sample.heater_duty)

        x = list(self.time_data)

        self.t_up_curve.setData(x, list(self.t_up_data))
        self.t_down_curve.setData(x, list(self.t_down_data))
        self.t_heater_curve.setData(x, list(self.t_heater_data))

        self.delta_curve.setData(x, list(self.delta_data))
        self.delta_corr_curve.setData(x, list(self.delta_corr_data))

        self.duty_curve.setData(x, list(self.duty_data))
        self.csv_buffer.append({
            "time_s": t,
            "t_up": sample.t_up,
            "t_down": sample.t_down,
            "t_heater": sample.t_heater,
            "delta_t": sample.delta_t,
            "delta_t_corr": sample.delta_t_corr,
            "heater_duty": sample.heater_duty,
        })

        # keep only last ~60 sec
        while len(self.csv_buffer) > 300:
            self.csv_buffer.pop(0)

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()

    def save_csv(self):
        if not self.csv_buffer:
            self.status_label.setText("Status: No data to save")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"thermal_flow_{timestamp}.csv"

        with open("save/" + filename, "w", newline="") as csvfile:
            writer = csv.DictWriter(
                csvfile,
                fieldnames=[
                    "time_s",
                    "t_up",
                    "t_down",
                    "t_heater",
                    "delta_t",
                    "delta_t_corr",
                    "heater_duty",
                ]
            )

            writer.writeheader()

            for row in self.csv_buffer:
                writer.writerow(row)

        self.status_label.setText(f"Status: Saved {filename}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ThermalFlowApp()
    window.show()
    sys.exit(app.exec())