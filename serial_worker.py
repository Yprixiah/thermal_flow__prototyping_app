import time
import serial
import serial.tools.list_ports
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from data_model import ThermalSample


class SerialWorker(QObject):
    sample_received = pyqtSignal(object)
    status_changed = pyqtSignal(str)
    raw_line_received = pyqtSignal(str)

    def __init__(self, port_name: str, baudrate: int = 115200):
        super().__init__()
        self.port_name = port_name
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False

    @staticmethod
    def available_ports():
        return [port.device for port in serial.tools.list_ports.comports()]

    @pyqtSlot()
    def run(self):
        try:
            self.status_changed.emit(f"Opening {self.port_name}...")

            self.serial_port = serial.Serial(
                self.port_name,
                self.baudrate,
                timeout=0.2
            )

            time.sleep(2.0)
            self.serial_port.reset_input_buffer()

            self.running = True
            self.status_changed.emit(f"Connected to {self.port_name}")

            while self.running:
                line = self.serial_port.readline().decode(errors="ignore").strip()

                if not line:
                    continue

                self.raw_line_received.emit(line)

                sample = self.parse_line(line)
                if sample is not None:
                    self.sample_received.emit(sample)

        except Exception as e:
            self.status_changed.emit(f"Serial error: {e}")

        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

            self.status_changed.emit("Disconnected")

    def stop(self):
        self.running = False

    @staticmethod
    def parse_line(line: str):
        if not line.startswith("DATA,"):
            return None

        try:
            parts = line.split(",")

            return ThermalSample(
                time_ms=int(parts[1]),
                t_up=float(parts[2]),
                t_down=float(parts[3]),
                t_heater=float(parts[4]),
                delta_t=float(parts[5]),
                delta_t_corr=float(parts[6]),
                heater_duty=int(parts[7]),
            )

        except Exception as e:
            print("Parse error:", e, "LINE:", line)
            return None