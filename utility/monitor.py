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
    QMessageBox, QFrame, QSlider, QGroupBox
)

import pyqtgraph as pg


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc


def format_at_command(cmd: str) -> str:
    crc = crc16_ccitt(cmd.encode("ascii"))
    return f"{cmd}*{crc:04X}\r\n"


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

    def write(self, data: bytes):
        if self.serial is not None and self._running:
            self.serial.write(data)

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


class SpeedController(QWidget):
    """Speed regulator widget that sends AT+SPD commands."""

    def __init__(self, write_callback):
        super().__init__()
        self._write = write_callback
        self._suppress_signal = False
        self._init_ui()

    def _init_ui(self):
        group = QGroupBox("Speed Control")
        layout = QHBoxLayout(group)

        layout.addWidget(QLabel("Target Speed:"))

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 200)
        self.slider.setValue(0)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.slider.valueChanged.connect(self._on_slider_changed)
        layout.addWidget(self.slider, stretch=2)

        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(0, 200)
        self.spinbox.setDecimals(1)
        self.spinbox.setSingleStep(5)
        self.spinbox.setValue(0)
        self.spinbox.setSuffix(" RPS")
        self.spinbox.setMinimumWidth(100)
        self.spinbox.valueChanged.connect(self._on_spinbox_changed)
        layout.addWidget(self.spinbox)

        self.group = group
        self.setLayout(layout)

    def set_enabled(self, enabled: bool):
        self.group.setEnabled(enabled)

    def _on_slider_changed(self, value: int):
        if self._suppress_signal:
            return
        self._suppress_signal = True
        self.spinbox.setValue(float(value))
        self._suppress_signal = False
        self._send_command()

    def _on_spinbox_changed(self, value: float):
        if self._suppress_signal:
            return
        self._suppress_signal = True
        self.slider.setValue(int(value))
        self._suppress_signal = False
        self._send_command()

    def _send_command(self):
        speed = self.spinbox.value()
        cmd = format_at_command(f"AT+SPD={speed:.1f}")
        self._write(cmd.encode("ascii"))


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
            if fields[0].startswith("+TM:"):
                fields[0] = fields[0][4:]
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
    """Captures +OSC bursts from AT oscilloscope output."""

    def __init__(self, write_callback=None):
        super().__init__()
        self._osc_samples: list[tuple[int, int]] = []
        self._osc_active = False
        self._write = write_callback
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        ctrl = QHBoxLayout()
        self.btn_capture = QPushButton("Start OSC")
        self.btn_capture.clicked.connect(self._toggle_capture)
        ctrl.addWidget(self.btn_capture)
        ctrl.addWidget(QLabel("Oscilloscope burst from +OSC stream"))
        ctrl.addStretch()
        layout.addLayout(ctrl)

        self.plot_currents = pg.PlotWidget()
        _style_plot(self.plot_currents, "Oscilloscope - Current", bottom_label="Sample Index")
        self.curve_meas_cur = self.plot_currents.plot(name="measured_current", pen=pg.mkPen("#00ffff", width=2))
        layout.addWidget(self.plot_currents, stretch=2)

        self.label_status = QLabel("Waiting for +OSC burst...")
        self.label_status.setStyleSheet("color: #888;")
        layout.addWidget(self.label_status, alignment=Qt.AlignmentFlag.AlignCenter)

    def _toggle_capture(self):
        if not self._osc_active:
            self._osc_active = True
            self.btn_capture.setText("Stop OSC")
            self.btn_capture.setStyleSheet("background-color: #ff4444; color: white;")
            self.label_status.setText("Capturing oscilloscope...")
            if self._write is not None:
                cmd = format_at_command("AT+OSC=1")
                self._write(cmd.encode("ascii"))
        else:
            self._osc_active = False
            self.btn_capture.setText("Start OSC")
            self.btn_capture.setStyleSheet("")
            if self._write is not None:
                cmd = format_at_command("AT+OSC=0")
                self._write(cmd.encode("ascii"))


    def reset(self):
        self._osc_samples.clear()
        self.curve_meas_cur.setData([], [])
        self.label_status.setText("Waiting for +OSC burst...")

    def add_data(self, sample_idx: int, current_scaled: int):
        self._osc_samples.append((sample_idx, current_scaled))
        if len(self._osc_samples) > 1100:
            self._osc_samples = self._osc_samples[-1000:]

    def finish_burst(self):
        if not self._osc_samples:
            self.label_status.setText("Empty burst received")
            return

        self._osc_samples.sort(key=lambda x: x[0])
        n = len(self._osc_samples)
        x = np.array([s[0] for s in self._osc_samples], dtype=np.float64)
        y = np.array([s[1] / 1000.0 for s in self._osc_samples], dtype=np.float64)

        self.curve_meas_cur.setData(x, y)
        self.plot_currents.setXRange(x[0], x[-1] + 10)
        self.label_status.setText(f"OSC burst: {n} samples")

    def add_line(self, line: str):
        if line == "":
            self.finish_burst()
            self._osc_samples.clear()
            return
        if line.startswith("+OSC:"):
            rest = line[5:]
            if "," in rest:
                parts = rest.split(",", 1)
                try:
                    idx = int(parts[0])
                    cur = int(parts[1])
                    self.add_data(idx, cur)
                except ValueError:
                    pass


class MonitorWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Open ECU Monitor")
        self.resize(1200, 900)

        self.worker_thread: QThread | None = None
        self.worker: SerialWorker | None = None
        self._connected = False

        self.tab_continuous = ContinuousTab()
        self.tab_waveform = CurrentWaveformTab(self._send_data)

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

        top_bar.addStretch()

        self.speed_ctrl = SpeedController(self._send_data)
        top_bar.addWidget(self.speed_ctrl, stretch=1)

        root.addLayout(top_bar)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFrameShadow(QFrame.Shadow.Sunken)
        root.addWidget(sep)

        tabs = QTabWidget()
        tabs.addTab(self.tab_continuous, "Continuous")
        tabs.addTab(self.tab_waveform, "Current Waveform")
        root.addWidget(tabs, stretch=1)

    def _send_data(self, data: bytes):
        if self.worker is not None:
            self.worker.write(data)

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
        self.speed_ctrl.set_enabled(True)
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
        self.speed_ctrl.set_enabled(False)
        self.tab_waveform._osc_active = False
        if hasattr(self.tab_waveform, 'btn_capture'):
            self.tab_waveform.btn_capture.setText("Start OSC")
            self.tab_waveform.btn_capture.setStyleSheet("")

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
        if line.startswith("+TM:") or ("+TM:" in line and ";" in line):
            self.tab_continuous.add_sample(line.split(";"))
        elif line.startswith("+OSC:"):
            if line == "+OSC:":
                self.tab_waveform.add_line("")
            else:
                self.tab_waveform.add_line(line)
        elif line.startswith("OK") or line.startswith("ERROR"):
            pass


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
        QSlider::groove:horizontal { border: 1px solid #555; height: 8px; background: #2a2a2a; border-radius: 4px; }
        QSlider::handle:horizontal { background: #4a9eff; width: 18px; margin-top: -6px; margin-bottom: -6px; border-radius: 9px; }
        QGroupBox { color: #ccc; border: 1px solid #555; margin-top: 6px; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
    """)
    window = MonitorWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
