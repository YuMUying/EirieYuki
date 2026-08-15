import math
import tempfile
import json
from pathlib import Path
import unittest

import numpy as np

from weld_seam.probe_alignment import estimate_probe_alignment, save_probe_alignment


class ProbeAlignmentTests(unittest.TestCase):
    def test_target_uses_capture_time_rail_position(self) -> None:
        points = np.array(
            [
                [0.20, 0.030, 0.0],
                [0.21, 0.031, 0.0],
                [0.22, 0.029, 0.0],
                [0.23, 0.030, 0.0],
                [0.24, 0.030, 0.0],
            ]
        )
        result = estimate_probe_alignment(points, rail_position_at_capture_m=0.020)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.lateral_error_m, 0.010)
        self.assertAlmostEqual(result.target_rail_position_m, 0.030)

    def test_too_few_points_is_invalid_and_json_safe(self) -> None:
        result = estimate_probe_alignment(
            np.array([[0.2, 0.01, 0.0]]),
            rail_position_at_capture_m=0.0,
        )
        self.assertFalse(result.valid)
        self.assertTrue(math.isfinite(result.seam_lateral_position_m))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "alignment.json"
            save_probe_alignment(
                path,
                result,
                capture_stamp_ns=123,
                task_id="task-1",
                sample_index=7,
            )
            payload = json.loads(path.read_text())
            self.assertFalse(payload["valid"])
            self.assertEqual(payload["capture_stamp_ns"], 123)
            self.assertEqual(payload["task_id"], "task-1")
            self.assertEqual(payload["sample_index"], 7)


if __name__ == "__main__":
    unittest.main()
