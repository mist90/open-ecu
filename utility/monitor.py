#!/usr/bin/env python3
"""Open ECU Serial Monitor - PyQt6 + pyqtgraph real-time BLDC telemetry viewer."""

import sys
import time
import bisect
import serial
import numpy as np
from serial.tools import list_ports

from PyQt6.QtCore import QObject, QThread, QTimer, pyqtSignal, Qt
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QComboBox, QDoubleSpinBox, QTabWidget,
    QMessageBox, QFrame
)

import pyqtgraph as pg


class SerialWorker(QObject):
    """Background thread that reads the serial port and emits parsed lines."""

    line_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool, str)

    def __init__(self, port: str, baud: int):
        super().__init__()
        self.port = port
        self.baud = baud
        self.serial: serial.Serial | None = None
        self._running = False

    def start_connection(self) -> bool:
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
            )
            self._running = True
            self.connection_status.emit(True, f"Connected ({self.port})")
            return True
        except serial.SerialException as exc:
            self.connection_status.emit(False, f"Error: {exc}")
            return False

    def stop_connection(self):
        self._running = False
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
        self.connection_status.emit(False, "Disconnected")

    def run(self):
        if self.serial is None:
            return
        buffer = ""
        while self._running:
            try:
                data = self.serial.read(self.serial.in_waiting or 1)
                if not data:
                    time.sleep(0.0005)
                    continue
                buffer += data.decode("ascii", errors="replace")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    stripped = line.strip()
                    self.line_received.emit(stripped if stripped else "")
            except Exception:
                break


def _style_plot(plot_widget: pg.PlotWidget, title: str = "", bottom_label: str = ""):
    plot_widget.setBackground("#1e1e1e")
    plot_widget.showGrid(x=True, y=True, alpha=0.3)
    plot_widget.addLegend()
    plot_widget.setTitle(title, color="#ffffff", size="12pt")
    if bottom_label:
        plot_widget.setLabel("bottom", bottom_label)
    plot_widget.setLabel("left", color="#cccccc")
    plot_widget.setLabel("bottom", color="#cccccc")


class ContinuousTab(QWidget):

    UPDATE_INTERVAL_MS = 16
    CONTROL_FREQ_HZ = 1000

    def __init__(self):
        super().__init__()
        self._buffer_size = 5.0
        self._dirty = False
        self._sample_count = 0

        self.t_data = []
        self.bus_voltage = []
        self.target_speed = []
        self.current_speed = []
        self.duty_cycle = []
        self.target_current = []
        self.measured_current = []

        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._update_plots)
        self._update_timer.start(self.UPDATE_INTERVAL_MS)

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        ctrl = QHBoxLayout()
        ctrl.addWidget(QLabel("Time Window (sec):"))
        self.window_spin = QDoubleSpinBox()
        self.window_spin.setRange(1, 60)
        self.window_spin.setSingleStep(0.5)
        self.window_spin.setValue(5)
        self.window_spin.valueChanged.connect(self._on_window_changed)
        ctrl.addWidget(self.window_spin)
        ctrl.addStretch()
        layout.addLayout(ctrl)

        self.plot_speeds = pg.PlotWidget()
        _style_plot(self.plot_speeds, "Speed", bottom_label="Time (s)")
        self.curve_spd_tgt  = self.plot_speeds.plot(name="target_speed",  pen=pg.mkPen("#ff0088", width=2))
        self.curve_spd_cur  = self.plot_speeds.plot(name="current_speed", pen=pg.mkPen("#ff4444", width=2))

        self.plot_currents = pg.PlotWidget()
        _style_plot(self.plot_currents, "Current", bottom_label="Time (s)")
        self.curve_cur_tgt  = self.plot_currents.plot(name="target_current",   pen=pg.mkPen("#aa00ff", width=2))
        self.curve_cur_meas = self.plot_currents.plot(name="measured_current", pen=pg.mkPen("#00ffff", width=2))

        self.plot_duty = pg.PlotWidget()
        _style_plot(self.plot_duty, "Duty Cycle", bottom_label="Time (s)")
        self.curve_duty = self.plot_duty.plot(name="duty_cycle", pen=pg.mkPen("#ffff00", width=2))

        self.plot_voltage = pg.PlotWidget()
        _style_plot(self.plot_voltage, "Bus Voltage", bottom_label="Time (s)")
        self.curve_bus_volt = self.plot_voltage.plot(name="bus_voltage", pen=pg.mkPen("#ffaa00", width=2))

        layout.addWidget(self.plot_speeds, stretch=2)
        layout.addWidget(self.plot_currents, stretch=2)
        layout.addWidget(self.plot_duty, stretch=1)
        layout.addWidget(self.plot_voltage, stretch=1)

    def _on_window_changed(self, value: float):
        self._buffer_size = value
        self._trim_buffer()

    def reset(self):
        self._sample_count = 0
        self.t_data.clear()
        self.bus_voltage.clear()
        self.target_speed.clear()
        self.current_speed.clear()
        self.duty_cycle.clear()
        self.target_current.clear()
        self.measured_current.clear()
        self._dirty = True

    def add_sample(self, fields: list[str]):
        if len(fields) != 8:
            return
        try:
            ts = float(fields[2])
            cs = float(fields[3])
            dc = float(fields[4])
            tc = float(fields[5])
            mc = float(fields[6])
            bv = float(fields[7])
        except ValueError:
            return

        rel_t = self._sample_count / self.CONTROL_FREQ_HZ

        self._sample_count += 1

        self.t_data.append(rel_t)
        self.bus_voltage.append(bv)
        self.target_speed.append(ts)
        self.current_speed.append(cs)
        self.duty_cycle.append(dc)
        self.target_current.append(tc)
        self.measured_current.append(mc)
        self._dirty = True

        self._trim_buffer()

    def _trim_buffer(self):
        if len(self.t_data) < 256:
            return
        cutoff = self.t_data[-1] - self._buffer_size
        idx = bisect.bisect_left(self.t_data, cutoff)
        if idx > 0:
            self.t_data = self.t_data[idx:]
            self.bus_voltage = self.bus_voltage[idx:]
            self.target_speed = self.target_speed[idx:]
            self.current_speed = self.current_speed[idx:]
            self.duty_cycle = self.duty_cycle[idx:]
            self.target_current = self.target_current[idx:]
            self.measured_current = self.measured_current[idx:]

    def _update_plots(self):
        if not self._dirty:
            return
        self._dirty = False

        if not self.t_data:
            self.curve_spd_tgt.setData([], [])
            self.curve_spd_cur.setData([], [])
            self.curve_cur_tgt.setData([], [])
            self.curve_cur_meas.setData([], [])
            self.curve_bus_volt.setData([], [])
            self.curve_duty.setData([], [])
            return

        tn = np.array(self.t_data, dtype=np.float64)
        self.curve_spd_tgt.setData(tn, np.array(self.target_speed, dtype=np.float64))
        self.curve_spd_cur.setData(tn, np.array(self.current_speed, dtype=np.float64))
        self.curve_cur_tgt.setData(tn, np.array(self.target_current, dtype=np.float64))
        self.curve_cur_meas.setData(tn, np.array(self.measured_current, dtype=np.float64))
        self.curve_bus_volt.setData(tn, np.array(self.bus_voltage, dtype=np.float64))
        self.curve_duty.setData(tn, np.array(self.duty_cycle, dtype=np.float64))

        t_min = max(0.0, self.t_data[-1] - self._buffer_size)
        t_max = self.t_data[-1] + 0.5
        self.plot_speeds.setXRange(t_min, t_max)
        self.plot_currents.setXRange(t_min, t_max)
        self.plot_duty.setXRange(t_min, t_max)
        self.plot_voltage.setXRange(t_min, t_max)


class CurrentWaveformTab(QWidget):
    """Captures a 1000-sample DEBUG_PWM_ISR burst and plots signals."""

    def __init__(self):
        super().__init__()
        self._lines_buffer: list[str] = []
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Fixed 1000 samples (burst on DEBUG_PWM_ISR)"), alignment=Qt.AlignmentFlag.AlignCenter)

        self.plot_currents = pg.PlotWidget()
        _style_plot(self.plot_currents, "PWM ISR - Current", bottom_label="Sample Index")
        self.curve_target_cur  = self.plot_currents.plot(name="target_current",   pen=pg.mkPen("#aa00ff", width=2))
        self.curve_meas_cur    = self.plot_currents.plot(name="measured_current", pen=pg.mkPen("#00ffff", width=2))
        layout.addWidget(self.plot_currents, stretch=2)

        self.plot_duty = pg.PlotWidget()
        _style_plot(self.plot_duty, "PWM ISR - Duty Cycle", bottom_label="Sample Index")
        self.curve_duty  = self.plot_duty.plot(name="duty_cycle", pen=pg.mkPen("#ffff00", width=2))
        layout.addWidget(self.plot_duty, stretch=1)

        self.plot_position = pg.PlotWidget()
        _style_plot(self.plot_position, "Commutation Position", bottom_label="Sample Index")
        self.curve_position  = self.plot_position.plot(name="current_position", pen=pg.mkPen("#00ff88", width=2))
        layout.addWidget(self.plot_position, stretch=1)

        self.label_status = QLabel("Waiting for burst...")
        self.label_status.setStyleSheet("color: #888;")
        layout.addWidget(self.label_status, alignment=Qt.AlignmentFlag.AlignCenter)

    def reset(self):
        self._lines_buffer.clear()
        self.curve_target_cur.setData([], [])
        self.curve_meas_cur.setData([], [])
        self.curve_duty.setData([], [])
        self.curve_position.setData([], [])
        self.label_status.setText("Waiting for burst...")

    def add_line(self, line: str):
        if line == "":
            self._process_burst()
            return
        self._lines_buffer.append(line)
        if len(self._lines_buffer) > 1100:
            self._lines_buffer.clear()

    def _process_burst(self):
        if len(self._lines_buffer) < 50:
            self._lines_buffer.clear()
            return

        duty = []
        tgt_cur = []
        meas_cur = []
        pos = []

        success = 0
        for raw in self._lines_buffer:
            parts = raw.split(",")
            if len(parts) != 4:
                continue
            try:
                duty.append(float(parts[0]))
                tgt_cur.append(float(parts[1]))
                meas_cur.append(float(parts[2]))
                pos.append(int(parts[3]))
                success += 1
            except ValueError:
                pass

        self._lines_buffer.clear()

        if success < 10:
            return

        n = len(duty)
        x = np.arange(n, dtype=np.float64)
        duty_arr = np.array(duty)
        tgt_cur_arr = np.array(tgt_cur)
        meas_cur_arr = np.array(meas_cur)
        pos_arr = np.array(pos, dtype=np.float64)

        self.curve_duty.setData(x, duty_arr)
        self.curve_target_cur.setData(x, tgt_cur_arr)
        self.curve_meas_cur.setData(x, meas_cur_arr)
        self.curve_position.setData(x, pos_arr)

        self.plot_currents.setXRange(0, max(n, 1000))
        self.plot_duty.setXRange(0, max(n, 1000))
        self.plot_position.setXRange(0, max(n, 1000))
        self.label_status.setText(f"Received burst: {n} samples")


class MonitorWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Open ECU Monitor")
        self.resize(1200, 800)

        self.worker_thread: QThread | None = None
        self.worker: SerialWorker | None = None
        self._connected = False

        self.tab_continuous = ContinuousTab()
        self.tab_waveform = CurrentWaveformTab()

        self._init_ui()
        self._populate_ports()

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        top_bar = QHBoxLayout()

        top_bar.addWidget(QLabel("Port:"))
        self.port_input = QLineEdit("/dev/ttyACM0")
        self.port_input.setMinimumWidth(140)
        top_bar.addWidget(self.port_input)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(200)
        self.port_combo.currentTextChanged.connect(self._on_port_selected)
        top_bar.addWidget(self.port_combo)

        top_bar.addWidget(QLabel("Baud:"))
        self.baud_input = QLineEdit("2000000")
        self.baud_input.setFixedWidth(100)
        top_bar.addWidget(self.baud_input)

        self.btn_connect = QPushButton("Start")
        self.btn_connect.setFixedWidth(100)
        self.btn_connect.clicked.connect(self._toggle_connection)
        top_bar.addWidget(self.btn_connect)

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #888;")
        top_bar.addWidget(self.status_label)

        root.addLayout(top_bar)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFrameShadow(QFrame.Shadow.Sunken)
        root.addWidget(sep)

        tabs = QTabWidget()
        tabs.addTab(self.tab_continuous, "Continuous")
        tabs.addTab(self.tab_waveform, "Current Waveform")
        root.addWidget(tabs, stretch=1)

    def _populate_ports(self):
        self.port_combo.clear()
        ports = sorted(p.device for p in list_ports.comports())
        if not ports:
            ports = ["/dev/ttyACM0"]
        p = self.port_input.text()
        if p and p not in ports:
            ports.insert(0, p)
        self.port_combo.addItems(ports)
        if p in ports:
            self.port_combo.setCurrentText(p)

    def _on_port_selected(self, text: str):
        self.port_input.setText(text)

    def _toggle_connection(self):
        if self._connected:
            self._stop()
        else:
            self._start()

    def _start(self):
        port = self.port_input.text().strip()
        if not port:
            QMessageBox.critical(self, "Error", "Port name is empty.")
            return

        try:
            baud = int(self.baud_input.text().strip())
        except ValueError:
            QMessageBox.critical(self, "Error", "Invalid baud rate.")
            return

        self.worker_thread = QThread()
        self.worker = SerialWorker(port, baud)
        self.worker.moveToThread(self.worker_thread)

        self.worker.line_received.connect(self._on_line)
        self.worker.connection_status.connect(self._on_status)
        self.worker_thread.finished.connect(self.worker.deleteLater)
        self.worker_thread.finished.connect(self.worker_thread.deleteLater)

        if not self.worker.start_connection():
            self._cleanup_thread()
            QMessageBox.critical(self, "Serial Error",
                                 f"Could not open {port}.\n\nCheck permissions and cable.")
            return

        self.worker_thread.started.connect(self.worker.run)
        self.worker_thread.start()

        self._connected = True
        self.btn_connect.setText("Stop")
        self.btn_connect.setStyleSheet("background-color: #ff4444; color: white;")
        self.tab_continuous.reset()
        self.tab_waveform.reset()
        self._populate_ports()

    def _stop(self):
        if self.worker is not None:
            self.worker.stop_connection()
        self._cleanup_thread()
        self._connected = False
        self.btn_connect.setText("Start")
        self.btn_connect.setStyleSheet("")
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: #888;")

    def _cleanup_thread(self):
        if self.worker_thread is not None:
            self.worker_thread.quit()
            self.worker_thread.wait(2000)
            self.worker_thread = None
        self.worker = None

    def _on_status(self, connected: bool, text: str):
        self.status_label.setText(text)
        if connected:
            self.status_label.setStyleSheet("color: #44ff44;")
        else:
            self.status_label.setStyleSheet("color: #888;")

    def _on_line(self, line: str):
        if ";" in line:
            self.tab_continuous.add_sample(line.split(";"))
        else:
            self.tab_waveform.add_line(line)


def main():
    pg.setConfigOptions(antialias=True, leftButtonPan=False)
    pg.setConfigOption("background", "#1e1e1e")
    pg.setConfigOption("foreground", "#cccccc")

    app = QApplication(sys.argv)
    app.setStyleSheet("""
        QTabWidget::pane { border: 1px solid #444; }
        QTabBar::tab { padding: 6px 16px; background: #333; color: #ccc; }
        QTabBar::tab:selected { background: #555; color: #fff; }
        QLineEdit, QSpinBox { background: #2a2a2a; color: #eee; border: 1px solid #555; }
        QComboBox { background: #2a2a2a; color: #eee; border: 1px solid #555; }
        QComboBox::drop-down { border: none; }
        QLabel { color: #ccc; }
    """)
    window = MonitorWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
