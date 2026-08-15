from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from weld_seam.rgbd_geometry import (
    CameraIntrinsics,
    DepthProjectionConfig,
    project_centerline_to_3d,
    recommended_d405_mount_transform,
    sample_depths,
    save_projection_outputs,
    validate_transform,
)
from project_paths import DEFAULT_ONNX_MODEL


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "d405_mount.yaml"
PROBE_CONFIG_PATH = PROJECT_ROOT / "config" / "probe_d405_mount.yaml"


def synthetic_intrinsics(width: int = 9, height: int = 7) -> CameraIntrinsics:
    return CameraIntrinsics(
        width=width,
        height=height,
        fx=100.0,
        fy=100.0,
        cx=(width - 1) / 2.0,
        cy=(height - 1) / 2.0,
    )


def projection_config(**changes: object) -> DepthProjectionConfig:
    config = DepthProjectionConfig(
        depth_scale_m_per_unit=0.001,
        min_depth_m=0.07,
        max_depth_m=0.50,
        neighborhood_radius_px=0,
        min_valid_neighbors=1,
        max_local_depth_mad_m=0.015,
        max_surface_distance_m=None,
        transform_base_from_camera=recommended_d405_mount_transform(),
    )
    return replace(config, **changes)


class TransformTests(unittest.TestCase):
    def test_recommended_transform_is_rigid_and_matches_yaml(self) -> None:
        expected = recommended_d405_mount_transform()
        validate_transform(expected)
        loaded = DepthProjectionConfig.load(CONFIG_PATH).transform_base_from_camera
        np.testing.assert_allclose(loaded, expected, atol=1e-9)
        self.assertAlmostEqual(float(np.linalg.det(expected[:3, :3])), 1.0, places=9)

    def test_invalid_rotation_is_rejected(self) -> None:
        invalid = np.eye(4)
        invalid[0, 0] = 2.0
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            validate_transform(invalid)

    def test_probe_camera_transform_tracks_rail_position(self) -> None:
        config = DepthProjectionConfig.load(PROBE_CONFIG_PATH)
        reference = config.transform_at_mount_position(0.0)
        shifted = config.transform_at_mount_position(0.025)
        np.testing.assert_allclose(
            shifted[:3, 3] - reference[:3, 3], [0, 0.025, 0]
        )

    def test_probe_camera_requires_capture_time_rail_position(self) -> None:
        config = DepthProjectionConfig.load(PROBE_CONFIG_PATH)
        with self.assertRaisesRegex(ValueError, "mount position"):
            config.transform_at_mount_position(None)


class ProjectionTests(unittest.TestCase):
    def test_realsense_inverse_brown_conrady_is_supported(self) -> None:
        intrinsics = replace(
            synthetic_intrinsics(),
            distortion_model="distortion.inverse_brown_conrady",
            distortion_coefficients=(0.08, -0.02, 0.001, -0.001, 0.005),
        )
        depth = np.full((intrinsics.height, intrinsics.width), 0.2, np.float32)
        point = np.array([[intrinsics.cx + 3.0, intrinsics.cy + 2.0]])
        result = project_centerline_to_3d(
            point, depth, intrinsics, projection_config()
        )
        self.assertEqual(len(result.points_camera_xyz_m), 1)
        self.assertTrue(np.all(np.isfinite(result.points_camera_xyz_m)))
        self.assertNotAlmostEqual(
            result.points_camera_xyz_m[0, 0], 3.0 / intrinsics.fx * 0.2, places=8
        )

    def test_center_ray_reaches_wall_at_expected_forward_position(self) -> None:
        intrinsics = synthetic_intrinsics()
        depth_to_wall = 0.200 / np.cos(np.deg2rad(35.0))
        depth = np.full((intrinsics.height, intrinsics.width), depth_to_wall, np.float32)
        point = np.array([[intrinsics.cx, intrinsics.cy]])
        result = project_centerline_to_3d(
            point, depth, intrinsics, projection_config(max_surface_distance_m=0.001)
        )
        self.assertEqual(len(result.source_indices), 1)
        self.assertAlmostEqual(result.points_base_xyz_m[0, 0], 0.275041, places=5)
        self.assertAlmostEqual(result.points_base_xyz_m[0, 1], 0.0, places=7)
        self.assertAlmostEqual(result.points_base_xyz_m[0, 2], 0.0, places=6)

    def test_image_right_maps_to_negative_base_y(self) -> None:
        intrinsics = synthetic_intrinsics()
        depth = np.full((intrinsics.height, intrinsics.width), 0.20, np.float32)
        points = np.array(
            [[intrinsics.cx - 2, intrinsics.cy], [intrinsics.cx + 2, intrinsics.cy]]
        )
        result = project_centerline_to_3d(points, depth, intrinsics, projection_config())
        self.assertGreater(result.points_base_xyz_m[0, 1], 0.0)
        self.assertLess(result.points_base_xyz_m[1, 1], 0.0)

    def test_neighborhood_median_recovers_center_depth_hole(self) -> None:
        intrinsics = synthetic_intrinsics()
        depth = np.full((intrinsics.height, intrinsics.width), 244, np.uint16)
        depth[int(intrinsics.cy), int(intrinsics.cx)] = 0
        config = projection_config(neighborhood_radius_px=1, min_valid_neighbors=3)
        values, valid, mads = sample_depths(
            depth, np.array([[intrinsics.cx, intrinsics.cy]]), config
        )
        self.assertTrue(valid[0])
        self.assertAlmostEqual(values[0], 0.244, places=6)
        self.assertAlmostEqual(mads[0], 0.0, places=9)

    def test_local_depth_mad_rejects_unstable_neighborhood(self) -> None:
        intrinsics = synthetic_intrinsics()
        depth = np.zeros((intrinsics.height, intrinsics.width), np.float32)
        y, x = int(intrinsics.cy), int(intrinsics.cx)
        depth[y - 1 : y + 2, x - 1 : x + 2] = np.array(
            [[0.10, 0.14, 0.18], [0.22, 0.26, 0.30], [0.34, 0.38, 0.42]],
            dtype=np.float32,
        )
        config = projection_config(
            neighborhood_radius_px=1,
            min_valid_neighbors=9,
            max_local_depth_mad_m=0.02,
        )
        _, valid, mads = sample_depths(
            depth, np.array([[intrinsics.cx, intrinsics.cy]]), config
        )
        self.assertFalse(valid[0])
        self.assertTrue(np.isnan(mads[0]))

    def test_surface_gate_rejects_point_far_from_wall(self) -> None:
        intrinsics = synthetic_intrinsics()
        depth = np.full((intrinsics.height, intrinsics.width), 0.15, np.float32)
        result = project_centerline_to_3d(
            np.array([[intrinsics.cx, intrinsics.cy]]),
            depth,
            intrinsics,
            projection_config(max_surface_distance_m=0.035),
        )
        self.assertEqual(len(result.source_indices), 0)
        self.assertEqual(result.rejected_surface_count, 1)
        self.assertEqual(result.rejected_depth_count, 0)

    def test_resolution_mismatch_is_rejected(self) -> None:
        intrinsics = synthetic_intrinsics()
        with self.assertRaisesRegex(ValueError, "Align depth to color"):
            project_centerline_to_3d(
                np.array([[1.0, 1.0]]),
                np.ones((4, 4), np.float32),
                intrinsics,
                projection_config(),
            )

    def test_empty_centerline_is_valid(self) -> None:
        intrinsics = synthetic_intrinsics()
        result = project_centerline_to_3d(
            np.empty((0, 2)),
            np.full((intrinsics.height, intrinsics.width), 0.2, np.float32),
            intrinsics,
            projection_config(),
        )
        self.assertEqual(result.points_base_xyz_m.shape, (0, 3))
        self.assertEqual(result.valid_ratio, 0.0)

    def test_projection_json_and_csv_are_written(self) -> None:
        intrinsics = synthetic_intrinsics()
        depth = np.full((intrinsics.height, intrinsics.width), 0.2, np.float32)
        result = project_centerline_to_3d(
            np.array(
                [
                    [intrinsics.cx - 1, intrinsics.cy],
                    [intrinsics.cx, intrinsics.cy],
                    [intrinsics.cx + 1, intrinsics.cy],
                ]
            ),
            depth,
            intrinsics,
            projection_config(),
        )
        with tempfile.TemporaryDirectory() as directory:
            save_projection_outputs(
                directory,
                result,
                intrinsics,
                projection_config(),
                "centerline.json",
                "depth.png",
            )
            output = Path(directory)
            payload = json.loads((output / "centerline_3d.json").read_text())
            self.assertEqual(payload["point_count"], 3)
            self.assertEqual(payload["segment_count"], 1)
            self.assertEqual(payload["base_frame"], "base_surface")
            self.assertTrue((output / "centerline_3d.csv").is_file())

    def test_missing_depth_splits_output_curve_segments(self) -> None:
        intrinsics = synthetic_intrinsics()
        depth = np.full((intrinsics.height, intrinsics.width), 0.2, np.float32)
        x, y = int(intrinsics.cx), int(intrinsics.cy)
        depth[y, x] = 0.0
        result = project_centerline_to_3d(
            np.array([[x - 1, y], [x, y], [x + 1, y]]),
            depth,
            intrinsics,
            projection_config(),
        )
        with tempfile.TemporaryDirectory() as directory:
            save_projection_outputs(
                directory,
                result,
                intrinsics,
                projection_config(),
                "centerline.json",
                "depth.png",
            )
            payload = json.loads(
                (Path(directory) / "centerline_3d.json").read_text()
            )
            self.assertEqual(payload["segment_count"], 2)
            self.assertEqual(
                [point["segment_id"] for point in payload["points"]], [0, 1]
            )


class ProjectionCliTests(unittest.TestCase):
    def test_onnx_rgbd_pipeline_end_to_end(self) -> None:
        width, height = 1280, 720
        model = DEFAULT_ONNX_MODEL
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            color_path = root / "color.png"
            depth_path = root / "depth.png"
            intrinsics_path = root / "intrinsics.json"
            output = root / "result"
            color = np.zeros((height, width, 3), np.uint8)
            cv2.line(color, (180, 360), (1100, 360), (210, 210, 210), 35)
            self.assertTrue(cv2.imwrite(str(color_path), color))
            self.assertTrue(
                cv2.imwrite(
                    str(depth_path), np.full((height, width), 244, np.uint16)
                )
            )
            intrinsics_path.write_text(
                json.dumps(
                    {
                        "width": width,
                        "height": height,
                        "fx": 650.0,
                        "fy": 650.0,
                        "cx": 639.5,
                        "cy": 359.5,
                    }
                ),
                encoding="utf-8",
            )
            completed = subprocess.run(
                [
                    sys.executable,
                    str(PROJECT_ROOT / "infer_rgbd.py"),
                    "--model",
                    str(model),
                    "--color",
                    str(color_path),
                    "--depth",
                    str(depth_path),
                    "--intrinsics",
                    str(intrinsics_path),
                    "--config",
                    str(CONFIG_PATH),
                    "--threshold",
                    "0.10",
                    "--min-area",
                    "20",
                    "--output",
                    str(output),
                ],
                cwd=PROJECT_ROOT,
                text=True,
                capture_output=True,
                check=True,
            )
            curve_2d = json.loads(
                (output / "curve_2d" / "centerline.json").read_text()
            )
            curve_3d = json.loads(
                (output / "curve_3d" / "centerline_3d.json").read_text()
            )
            self.assertGreater(curve_2d["point_count"], 10)
            self.assertEqual(curve_3d["point_count"], curve_2d["point_count"])
            self.assertEqual(curve_3d["segment_count"], 1)
            self.assertAlmostEqual(curve_3d["valid_ratio"], 1.0)
            self.assertIn("valid_ratio=1.000", completed.stdout)

    def test_synthetic_depth_cli_end_to_end(self) -> None:
        width, height = 9, 7
        intrinsics = synthetic_intrinsics(width, height)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            centerline = root / "centerline.json"
            depth_path = root / "depth.png"
            intrinsics_path = root / "intrinsics.json"
            output = root / "output"
            points = [
                {"x_px": intrinsics.cx - 1, "y_px": intrinsics.cy},
                {"x_px": intrinsics.cx, "y_px": intrinsics.cy},
                {"x_px": intrinsics.cx + 1, "y_px": intrinsics.cy},
            ]
            centerline.write_text(json.dumps({"points": points}), encoding="utf-8")
            intrinsics_path.write_text(
                json.dumps(
                    {
                        "width": width,
                        "height": height,
                        "fx": intrinsics.fx,
                        "fy": intrinsics.fy,
                        "cx": intrinsics.cx,
                        "cy": intrinsics.cy,
                    }
                ),
                encoding="utf-8",
            )
            self.assertTrue(
                cv2.imwrite(str(depth_path), np.full((height, width), 244, np.uint16))
            )
            completed = subprocess.run(
                [
                    sys.executable,
                    str(PROJECT_ROOT / "project_rgbd.py"),
                    "--centerline",
                    str(centerline),
                    "--depth",
                    str(depth_path),
                    "--intrinsics",
                    str(intrinsics_path),
                    "--config",
                    str(CONFIG_PATH),
                    "--output",
                    str(output),
                ],
                cwd=PROJECT_ROOT,
                text=True,
                capture_output=True,
                check=True,
            )
            payload = json.loads((output / "centerline_3d.json").read_text())
            self.assertEqual(payload["point_count"], 3)
            self.assertIn("points_2d=3 points_3d=3", completed.stdout)


if __name__ == "__main__":
    unittest.main()
