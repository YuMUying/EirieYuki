from __future__ import annotations

import unittest

from weld_seam.streaming import LatestValueSlot, RateGate, RollingLatency, TimedScalarBuffer


class LatestValueSlotTests(unittest.TestCase):
    def test_newest_value_replaces_pending_work(self) -> None:
        slot: LatestValueSlot[str] = LatestValueSlot()
        slot.put("old")
        slot.put("new")
        self.assertEqual(slot.take(), (2, "new"))
        self.assertEqual(slot.dropped, 1)
        self.assertIsNone(slot.take())


class RateGateTests(unittest.TestCase):
    def test_limits_start_rate(self) -> None:
        gate = RateGate(10.0)
        self.assertEqual(gate.delay_s(1.0), 0.0)
        gate.mark_started(1.0)
        self.assertAlmostEqual(gate.delay_s(1.05), 0.05)
        self.assertEqual(gate.delay_s(1.101), 0.0)
        gate.mark_started(1.101)
        self.assertAlmostEqual(gate.delay_s(1.15), 0.05)


class RollingLatencyTests(unittest.TestCase):
    def test_reports_percentiles_and_rate(self) -> None:
        latency = RollingLatency(10)
        for index in range(5):
            latency.add(0.010 + index * 0.001, 0.020, 1.0 + index * 0.1)
        snapshot = latency.snapshot()
        self.assertEqual(snapshot.samples, 5)
        self.assertAlmostEqual(snapshot.processing_mean_ms, 12.0)
        self.assertGreater(snapshot.processing_p95_ms, 13.0)
        self.assertAlmostEqual(snapshot.effective_hz, 10.0)


class TimedScalarBufferTests(unittest.TestCase):
    def test_interpolates_capture_time_value(self) -> None:
        values = TimedScalarBuffer()
        values.add(1_000_000_000, -0.01)
        values.add(1_100_000_000, 0.01)
        self.assertAlmostEqual(values.interpolate(1_050_000_000, 0.1), 0.0)
        self.assertIsNone(values.interpolate(2_000_000_000, 0.1))


if __name__ == "__main__":
    unittest.main()
