#!/usr/bin/env python3
"""Protocol-to-plot regressions; requires the monitor's PyQt6/pyqtgraph environment.

Run: QT_QPA_PLATFORM=offscreen python3 tests/test_monitor/test_monitor.py
"""

import binascii
import os
from pathlib import Path
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utility"))

import numpy as np
from PyQt6.QtTest import QSignalSpy
from PyQt6.QtWidgets import QApplication

from monitor import CurrentWaveformTab, MonitorWindow, OSC_BUFFER_SIZE


def sample(index, current=1100, legacy=False):
    fields = [index, current, 1000, 50, index % 6]
    if not legacy:
        fields += [20, current, index % 360]
    return "+OSC:" + ",".join(map(str, fields))


class WaveformTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tab = CurrentWaveformTab()

    def tearDown(self):
        self.tab.close()

    def feed(self, skip=(), current=1100, legacy=False, terminate=True):
        for i in range(OSC_BUFFER_SIZE):
            if i not in skip:
                self.tab.add_line(sample(i, current, legacy))
        if terminate:
            self.tab.add_line("+OSC:")

    def test_complete_burst_and_distinct_error_rms(self):
        self.feed()
        x, y = self.tab.curve_meas_cur.getData()
        np.testing.assert_array_equal(x, np.arange(OSC_BUFFER_SIZE))
        np.testing.assert_allclose(y, 1.1)
        # A constant tracking error has zero ripple, but nonzero error RMS.
        self.assertIn("sd 0.0000 A", self.tab.label_status.text())
        self.assertIn("error RMS 0.1000 A", self.tab.label_status.text())
        self.assertIn("1024/1024", self.tab.label_status.text())

    def test_lost_uart_chunk_breaks_every_curve(self):
        self.feed(skip=range(320, 352))
        for curve in (self.tab.curve_meas_cur, self.tab.curve_tgt_cur,
                      self.tab.curve_id, self.tab.curve_iq, self.tab.curve_angle,
                      self.tab.curve_duty, self.tab.curve_position):
            x, y = curve.getData()
            np.testing.assert_array_equal(x, np.arange(OSC_BUFFER_SIZE))
            self.assertTrue(np.isnan(y[320:352]).all())
            self.assertEqual(np.isfinite(y).sum(), 992)
            # The rendered path must start a new subpath after the missing span.
            path = curve.curve.getPath()
            starts = [path.elementAt(i).x for i in range(path.elementCount())
                      if path.elementAt(i).isMoveTo()]
            self.assertIn(352, starts)
        self.assertIn("missing 32 (320–351)", self.tab.label_status.text())
        self.assertIn("statistics unavailable", self.tab.label_status.text())

    def test_missing_prefix_and_suffix_are_not_hidden_by_autorange(self):
        self.feed(skip=set(range(20)) | set(range(1000, 1024)))
        x, y = self.tab.curve_meas_cur.getData()
        self.assertEqual((x[0], x[-1]), (0, 1023))
        self.assertTrue(np.isnan(y[:20]).all())
        self.assertTrue(np.isnan(y[1000:]).all())
        self.assertIn("missing 44 (0–19, 1000–1023)", self.tab.label_status.text())

    def test_lost_terminator_does_not_merge_acquisitions(self):
        self.feed(current=8000, terminate=False)
        self.feed(current=1100)
        _, y = self.tab.curve_meas_cur.getData()
        np.testing.assert_allclose(y, 1.1)
        self.assertIn("1024/1024", self.tab.label_status.text())
        self.assertIn("incomplete bursts: 1", self.tab.label_status.text())

    def test_stop_start_discards_unterminated_capture(self):
        self.tab._toggle_capture()
        for i in range(100):
            self.tab.add_line(sample(i, 8000))
        self.tab._toggle_capture()
        self.tab._toggle_capture()
        self.feed()
        _, y = self.tab.curve_meas_cur.getData()
        np.testing.assert_allclose(y, 1.1)
        self.assertIn("incomplete bursts: 0", self.tab.label_status.text())

    def test_truncated_and_invalid_lines_are_reported(self):
        for i in range(OSC_BUFFER_SIZE):
            line = sample(i)
            if i == 100:
                line = "+OSC:100,1100,1000,50,1,20"
            elif i == 101:
                line = "+OSC:101,bad,1000,50,1,20,1100,0"
            self.tab.add_line(line)
        self.tab.add_line("+OSC:")
        self.assertIn("missing 2 (100–101)", self.tab.label_status.text())
        self.assertIn("malformed 2", self.tab.label_status.text())
        self.assertIn("statistics unavailable", self.tab.label_status.text())

    def test_duplicate_is_not_a_second_measurement(self):
        for i in range(OSC_BUFFER_SIZE):
            self.tab.add_line(sample(i))
            if i == 100:
                self.tab.add_line(sample(i, 8000))
        self.tab.add_line("+OSC:")
        _, y = self.tab.curve_meas_cur.getData()
        np.testing.assert_allclose(y, 1.1)
        self.assertIn("duplicates 1", self.tab.label_status.text())

    def test_legacy_five_field_burst_and_interleaved_telemetry(self):
        for i in range(OSC_BUFFER_SIZE):
            self.tab.add_line(sample(i, legacy=True))
            self.tab.add_line("+TM:0;0;0;0;0;0;0;40;0;20")
        self.tab.add_line("")
        _, y = self.tab.curve_iq.getData()
        np.testing.assert_array_equal(y, np.zeros(OSC_BUFFER_SIZE))
        self.assertIn("1024/1024", self.tab.label_status.text())
        self.assertIn("incomplete bursts: 0", self.tab.label_status.text())


class ControlConnectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = MonitorWindow()
        self.master, self.slave = os.openpty()
        os.set_blocking(self.master, False)
        self.window.port_input.setText(os.ttyname(self.slave))
        self.panel = self.window.control_panel

    def tearDown(self):
        self.window._stop()
        self.window.close()
        if self.master is not None:
            os.close(self.master)
        os.close(self.slave)

    def outbound(self):
        data = bytearray()
        while True:
            try:
                data.extend(os.read(self.master, 4096))
            except BlockingIOError:
                break
        commands = []
        for line in bytes(data).splitlines():
            command, crc = line.rsplit(b'*', 1)
            self.assertEqual(int(crc, 16), binascii.crc_hqx(command, 0xffff))
            commands.append(command.decode('ascii'))
        return commands

    def connect_read_only(self):
        self.window._start()
        self.assertTrue(self.panel.isEnabled())
        self.assertEqual(self.window.port_input.text(), os.ttyname(self.slave))
        self.assertCountEqual(self.outbound(), ['AT+MAXVALS?', 'AT+STATUS?', 'AT+DMODE?',
                                              'AT+ACC?', 'AT+SPID?', 'AT+CPID?', 'AT+ALGO?', 'AT+FPID?'])

    def test_read_only_startup_feedback_and_current_regulated_modes(self):
        self.assertFalse(self.panel.isEnabled())
        self.connect_read_only()
        for mode, controls in ((0, (True, True)), (1, (True, False)), (2, (False, True))):
            self.window._on_line(f'+STATUS:{mode},1,6.25,-1.5,0.1,42.0,1,0,0,0')
            self.window._on_line('+DMODE:0')
            self.assertEqual((self.panel.speed_ctrl.group.isEnabled(), self.panel.current_ctrl.group.isEnabled()), controls)
        self.window._on_line('+TM:0;0;3.5;6.25;0.1;-1.5;0;42;0;23')
        self.assertEqual(self.panel.speed_ctrl.spinbox.value(), 3.5)  # target, not feedback
        self.assertEqual(self.panel.current_ctrl.spinbox.value(), -1.5)
        self.window._on_line('+MAXVALS:2,-1,45,60,1,85')  # Clamping either widget must not echo a drive command.
        self.assertEqual((self.panel.speed_ctrl.spinbox.value(), self.panel.current_ctrl.spinbox.value()), (2, -1))
        self.assertEqual(self.panel.current_ctrl.spinbox.maximum(), 6)
        self.assertFalse(self.outbound())
        self.window._on_line('+MODE:0')
        self.panel.speed_ctrl.spinbox.setValue(1.75)
        self.panel.current_ctrl.spinbox.setValue(2.5)
        self.assertEqual(self.outbound(), ['AT+SPD=1.75', 'AT+CUR=2.5'])
        self.window._on_line('+DMODE:2')
        self.assertFalse(self.panel.speed_ctrl.group.isEnabled() or self.panel.current_ctrl.group.isEnabled())

        self.window._stop()
        self.connect_read_only()
        lines = QSignalSpy(self.window.worker.line_received)
        os.write(self.master, b'+TM:0;0;0.75;1.25;0;0;0;42;23;23\r\n')
        self.assertTrue(lines or lines.wait(2000))
        self.app.processEvents()
        self.assertEqual(self.panel.speed_ctrl.spinbox.value(), 0.75)
        self.assertFalse(self.outbound())

    def test_unplug_disables_commands(self):
        self.window._start()
        status = QSignalSpy(self.window.worker.connection_status)
        os.close(self.master)
        self.master = None
        if not status:
            self.assertTrue(status.wait(2000))
        self.app.processEvents()
        self.assertFalse(self.window._connected)
        self.assertFalse(self.panel.isEnabled() or self.window.tab_waveform.btn_capture.isEnabled())


if __name__ == "__main__":
    unittest.main()
