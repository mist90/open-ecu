#!/usr/bin/env python3
"""Protocol-to-plot regressions; requires the monitor's PyQt6/pyqtgraph environment.

Run: QT_QPA_PLATFORM=offscreen python3 tests/test_monitor/test_monitor.py
"""

import os
from pathlib import Path
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utility"))

import numpy as np
from PyQt6.QtWidgets import QApplication

from monitor import CurrentWaveformTab, OSC_BUFFER_SIZE


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


if __name__ == "__main__":
    unittest.main()
