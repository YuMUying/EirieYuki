from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from weld_seam.centerline import extract_centerline, extract_centerlines, skeletonize
from weld_seam.io_utils import save_centerline_outputs
from weld_seam.preprocess import letterbox, restore_points, restore_probability


class CenterlineTests(unittest.TestCase):
    def test_extracts_multiple_spatially_separated_centerlines(self) -> None:
        mask = np.zeros((160, 200), dtype=np.uint8)
        cv2.line(mask, (30, 10), (30, 150), 255, 9)
        cv2.line(mask, (150, 10), (150, 150), 255, 9)
        results = extract_centerlines(
            mask, min_area=100, close_kernel=3, minimum_separation_pixels=30
        )
        self.assertEqual(len(results), 2)
        centers = sorted(float(np.median(result.points_xy[:, 0])) for result in results)
        self.assertAlmostEqual(centers[0], 30.0, delta=2.0)
        self.assertAlmostEqual(centers[1], 150.0, delta=2.0)

    def test_multi_centerline_separation_filter(self) -> None:
        mask = np.zeros((180, 180), dtype=np.uint8)
        cv2.line(mask, (15, 90), (165, 90), 255, 7)
        cv2.line(mask, (90, 15), (90, 165), 255, 7)
        results = extract_centerlines(
            mask,
            min_area=100,
            close_kernel=3,
            minimum_separation_pixels=25,
            minimum_branch_straightness=0.80,
        )
        self.assertEqual(len(results), 2)
        spans = sorted(
            (float(np.ptp(result.points_xy[:, 0])), float(np.ptp(result.points_xy[:, 1])))
            for result in results
        )
        self.assertTrue(any(x_span > 120 and y_span < 10 for x_span, y_span in spans))
        self.assertTrue(any(y_span > 120 and x_span < 10 for x_span, y_span in spans))

    def test_nearby_parallel_lines_can_be_collapsed_by_distance(self) -> None:
        mask = np.zeros((120, 120), dtype=np.uint8)
        cv2.line(mask, (40, 5), (40, 115), 255, 5)
        cv2.line(mask, (50, 5), (50, 115), 255, 5)
        results = extract_centerlines(
            mask,
            min_area=50,
            close_kernel=1,
            minimum_separation_pixels=20,
        )
        self.assertEqual(len(results), 1)

    def test_horizontal_band_returns_ordered_curve(self) -> None:
        mask = np.zeros((160, 240), dtype=np.uint8)
        cv2.line(mask, (20, 80), (220, 80), 255, 19)
        result = extract_centerline(
            mask, min_area=20, close_kernel=3, point_spacing=10, smooth_window=5
        )
        self.assertGreaterEqual(len(result.points_xy), 18)
        self.assertLessEqual(len(result.points_xy), 23)
        self.assertLess(result.points_xy[0, 0], result.points_xy[-1, 0])
        self.assertLess(float(np.max(np.abs(result.points_xy[:, 1] - 80))), 2.5)
        spacings = np.linalg.norm(np.diff(result.points_xy, axis=0), axis=1)
        self.assertGreater(float(np.min(spacings)), 7.0)
        self.assertLess(float(np.max(spacings)), 12.0)

    def test_curved_band_ignores_small_noise(self) -> None:
        mask = np.zeros((220, 260), dtype=np.uint8)
        curve = np.array(
            [(x, int(100 + 35 * np.sin(x / 55.0))) for x in range(20, 241)],
            dtype=np.int32,
        )
        cv2.polylines(mask, [curve.reshape(-1, 1, 2)], False, 255, 17)
        cv2.circle(mask, (245, 20), 3, 255, -1)
        result = extract_centerline(mask, min_area=100, point_spacing=12)
        self.assertGreater(len(result.points_xy), 15)
        self.assertLess(result.points_xy[0, 0], result.points_xy[-1, 0])
        self.assertTrue(np.all(result.points_xy[:, 1] > 40))

    def test_empty_mask_is_valid_empty_result(self) -> None:
        result = extract_centerline(np.zeros((64, 64), dtype=np.uint8))
        self.assertEqual(result.points_xy.shape, (0, 2))
        self.assertEqual(int(result.skeleton.sum()), 0)

    def test_thinning_is_single_pixel_for_simple_band(self) -> None:
        mask = np.zeros((80, 120), dtype=np.uint8)
        mask[30:50, 10:110] = 255
        skeleton = skeletonize(mask)
        self.assertGreater(np.count_nonzero(skeleton), 70)
        self.assertLess(np.count_nonzero(skeleton), 110)

    def test_output_contains_image_center_relative_coordinates(self) -> None:
        mask = np.zeros((101, 201), dtype=np.uint8)
        points = np.array([[100.0, 50.0], [200.0, 100.0]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            save_centerline_outputs(
                directory,
                "synthetic.png",
                mask.astype(np.float32),
                mask,
                mask,
                points,
                1.0,
                111.8,
            )
            payload = json.loads(
                (Path(directory) / "centerline.json").read_text(encoding="utf-8")
            )
            self.assertEqual(payload["relative_center_range"], "[-1,1]_x_right_y_down")
            self.assertAlmostEqual(payload["points"][0]["x_relative_center"], 0.0)
            self.assertAlmostEqual(payload["points"][0]["y_relative_center"], 0.0)
            self.assertAlmostEqual(payload["points"][1]["x_relative_center"], 1.0)
            self.assertAlmostEqual(payload["points"][1]["y_relative_center"], 1.0)
            self.assertTrue((Path(directory) / "edge.png").is_file())


class PreprocessTests(unittest.TestCase):
    def test_letterbox_round_trip_shape(self) -> None:
        image = np.zeros((120, 320, 3), dtype=np.uint8)
        prepared, transform = letterbox(image, 256)
        self.assertEqual(prepared.shape, (256, 256, 3))
        probability = np.ones((256, 256), dtype=np.float32)
        restored = restore_probability(probability, transform)
        self.assertEqual(restored.shape, (120, 320))
        self.assertTrue(np.allclose(restored, 1.0))

    def test_resized_points_restore_to_original_pixels(self) -> None:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        _, transform = letterbox(image, 192)
        points = np.array([[0.0, 0.0], [191.0, 143.0]], dtype=np.float32)
        restored = restore_points(points, transform)
        np.testing.assert_allclose(
            restored,
            [[1.1666667, 1.1666667], [637.8333, 477.8333]],
            atol=1e-3,
        )


if __name__ == "__main__":
    unittest.main()
