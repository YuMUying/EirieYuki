from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from wcr_planning_msgs.msg import (
    DeviceState,
    ProbeRailState,
    VisionTiming,
    WeldSeamCandidate,
    WeldSeamCandidateArray,
)

from weld_seam.centerline import extract_centerlines
from weld_seam.inference import OnnxSegmenter
from weld_seam.preprocess import restore_points
from weld_seam.rgbd_geometry import (
    CameraIntrinsics,
    DepthProjectionConfig,
    project_centerline_to_3d,
)
from weld_seam.streaming import LatestValueSlot, RateGate, RollingLatency, TimedScalarBuffer

from .image_conversion import image_to_array


@dataclass(frozen=True)
class RgbdMessages:
    color: Image
    depth: Image


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class RgbdWeldLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("rgbd_weld_localizer")
        self.declare_parameter("camera_role", "probe")
        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_config_path", "")
        self.declare_parameter("color_topic", "color/image_raw")
        self.declare_parameter("depth_topic", "aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "color/camera_info")
        self.declare_parameter("rail_state_topic", "/wcr/probe_rail/state")
        self.declare_parameter("candidate_topic", "weld_seam_candidates")
        self.declare_parameter("timing_topic", "vision_timing")
        self.declare_parameter("state_topic", "vision_state")
        self.declare_parameter("target_rate_hz", 30.0)
        self.declare_parameter("maximum_pair_offset_s", 0.012)
        self.declare_parameter("maximum_capture_age_s", 0.10)
        self.declare_parameter("maximum_processing_s", 0.033)
        self.declare_parameter("maximum_rail_offset_s", 0.02)
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("minimum_area", 100)
        self.declare_parameter("close_kernel", 7)
        self.declare_parameter("point_spacing_px", 8.0)
        self.declare_parameter("smooth_window", 7)
        self.declare_parameter("minimum_separation_px", 12.0)
        self.declare_parameter("maximum_centerlines", 8)
        self.declare_parameter("minimum_candidate_confidence", 0.50)
        self.declare_parameter("onnx_providers", ["CPUExecutionProvider"])
        self.declare_parameter("onnx_intra_op_threads", 0)
        self.declare_parameter("onnx_inter_op_threads", 0)
        self.declare_parameter("task_id", "")

        value = lambda name: self.get_parameter(name).value
        self.role = str(value("camera_role"))
        if self.role not in ("top", "probe"):
            raise ValueError("camera_role must be top or probe")
        model_path = Path(str(value("model_path"))).expanduser()
        config_path = Path(str(value("camera_config_path"))).expanduser()
        if not model_path.is_file() or not config_path.is_file():
            raise FileNotFoundError("model_path and camera_config_path must be existing files")
        self.segmenter = OnnxSegmenter(
            model_path,
            providers=list(value("onnx_providers")),
            intra_op_threads=int(value("onnx_intra_op_threads")),
            inter_op_threads=int(value("onnx_inter_op_threads")),
        )
        self.projection_config = DepthProjectionConfig.load(config_path)
        self.threshold = float(value("threshold"))
        self.minimum_area = int(value("minimum_area"))
        self.close_kernel = int(value("close_kernel"))
        self.point_spacing = float(value("point_spacing_px"))
        self.smooth_window = int(value("smooth_window"))
        self.minimum_separation = float(value("minimum_separation_px"))
        self.maximum_centerlines = int(value("maximum_centerlines"))
        self.minimum_confidence = float(value("minimum_candidate_confidence"))
        self.maximum_pair_offset_ns = int(float(value("maximum_pair_offset_s")) * 1e9)
        self.maximum_capture_age_s = float(value("maximum_capture_age_s"))
        self.maximum_processing_s = float(value("maximum_processing_s"))
        self.maximum_rail_offset_s = float(value("maximum_rail_offset_s"))
        self.task_id = str(value("task_id"))

        self.frames: LatestValueSlot[RgbdMessages] = LatestValueSlot()
        self.rate_gate = RateGate(float(value("target_rate_hz")))
        self.latency = RollingLatency(120)
        self.rail_positions = TimedScalarBuffer(3.0)
        self._sync_lock = threading.Lock()
        self._latest_color: Image | None = None
        self._latest_depth: Image | None = None
        self._last_pair: tuple[int, int] | None = None
        self._intrinsics_lock = threading.Lock()
        self._intrinsics: CameraIntrinsics | None = None
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._processed = 0
        self._superseded = 0
        self._sample_index = 0
        self._last_error = "waiting_for_rgbd"

        callback_group = ReentrantCallbackGroup()
        self.candidate_publisher = self.create_publisher(
            WeldSeamCandidateArray, str(value("candidate_topic")), 10
        )
        self.timing_publisher = self.create_publisher(
            VisionTiming, str(value("timing_topic")), 10
        )
        self.state_publisher = self.create_publisher(
            DeviceState, str(value("state_topic")), 10
        )
        self.create_subscription(
            Image,
            str(value("color_topic")),
            self._color_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            Image,
            str(value("depth_topic")),
            self._depth_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            CameraInfo,
            str(value("camera_info_topic")),
            self._camera_info_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        if self.role == "probe":
            self.create_subscription(
                ProbeRailState,
                str(value("rail_state_topic")),
                self._rail_callback,
                qos_profile_sensor_data,
                callback_group=callback_group,
            )
        self.create_timer(1.0, self._publish_state, callback_group=callback_group)
        self._worker = threading.Thread(
            target=self._worker_loop, name=f"{self.role}-vision-worker", daemon=True
        )
        self._worker.start()
        self.get_logger().info(
            f"Realtime {self.role} localizer ready; providers={self.segmenter.session.get_providers()}"
        )

    def _color_callback(self, message: Image) -> None:
        with self._sync_lock:
            self._latest_color = message
            self._try_pair_locked()

    def _depth_callback(self, message: Image) -> None:
        with self._sync_lock:
            self._latest_depth = message
            self._try_pair_locked()

    def _try_pair_locked(self) -> None:
        if self._latest_color is None or self._latest_depth is None:
            return
        color_stamp = stamp_ns(self._latest_color.header.stamp)
        depth_stamp = stamp_ns(self._latest_depth.header.stamp)
        pair = (color_stamp, depth_stamp)
        if pair == self._last_pair or abs(color_stamp - depth_stamp) > self.maximum_pair_offset_ns:
            return
        self._last_pair = pair
        self.frames.put(RgbdMessages(self._latest_color, self._latest_depth))
        self._latest_color = None
        self._latest_depth = None
        self._wake.set()

    def _camera_info_callback(self, message: CameraInfo) -> None:
        distortion = tuple(float(item) for item in message.d)
        intrinsics = CameraIntrinsics(
            width=int(message.width),
            height=int(message.height),
            fx=float(message.k[0]),
            fy=float(message.k[4]),
            cx=float(message.k[2]),
            cy=float(message.k[5]),
            distortion_model=str(message.distortion_model or "none"),
            distortion_coefficients=distortion,
        )
        with self._intrinsics_lock:
            self._intrinsics = intrinsics

    def _rail_callback(self, message: ProbeRailState) -> None:
        try:
            self.rail_positions.add(stamp_ns(message.header.stamp), float(message.position_m))
        except ValueError as error:
            self.get_logger().warn(str(error), throttle_duration_sec=1.0)

    def _worker_loop(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(0.1)
            self._wake.clear()
            queued = self.frames.take()
            if queued is None:
                continue
            delay_s = self.rate_gate.delay_s(time.monotonic())
            deadline = time.monotonic() + delay_s
            while not self._stop.is_set() and time.monotonic() < deadline:
                self._wake.wait(deadline - time.monotonic())
                self._wake.clear()
                replacement = self.frames.take()
                if replacement is not None:
                    self._superseded += 1
                    queued = replacement
            self.rate_gate.mark_started(time.monotonic())
            _, frame = queued
            try:
                self._process(frame)
            except Exception as error:
                self._last_error = f"processing_error:{type(error).__name__}:{error}"
                self.get_logger().error(self._last_error, throttle_duration_sec=1.0)

    def _process(self, frame: RgbdMessages) -> None:
        started = time.perf_counter()
        capture_ns = stamp_ns(frame.color.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        capture_age_s = (now_ns - capture_ns) / 1e9
        if capture_ns <= 0 or capture_age_s < -0.01 or capture_age_s > self.maximum_capture_age_s:
            self._last_error = "stale_or_invalid_capture_timestamp"
            return
        with self._intrinsics_lock:
            intrinsics = self._intrinsics
        if intrinsics is None:
            self._last_error = "camera_info_unavailable"
            return
        rail_position = None
        if self.role == "probe":
            rail_position = self.rail_positions.interpolate(
                capture_ns, self.maximum_rail_offset_s
            )
            if rail_position is None:
                self._last_error = "capture_time_rail_position_unavailable"
                return
        color = image_to_array(frame.color)
        depth = image_to_array(frame.depth)
        if color.shape[:2] != depth.shape[:2]:
            raise ValueError("aligned color and depth dimensions differ")
        probability, transform = self.segmenter.predict_resized(color)
        scale = min(
            transform.resized_width / transform.original_width,
            transform.resized_height / transform.original_height,
        )
        scaled_area = max(1, int(round(self.minimum_area * scale * scale)))
        scaled_kernel = max(1, int(round(self.close_kernel * scale)))
        scaled_spacing = max(1.0, self.point_spacing * scale)
        scaled_separation = max(1.0, self.minimum_separation * scale)
        mask = np.where(probability >= self.threshold, 255, 0).astype(np.uint8)
        centerlines = extract_centerlines(
            mask,
            min_area=scaled_area,
            close_kernel=scaled_kernel,
            point_spacing=scaled_spacing,
            smooth_window=self.smooth_window,
            minimum_separation_pixels=scaled_separation,
            maximum_centerlines=self.maximum_centerlines,
        )
        output = WeldSeamCandidateArray()
        output.header = frame.color.header
        output.header.frame_id = self.projection_config.base_frame
        output.camera_frame = frame.color.header.frame_id or self.projection_config.camera_frame
        output.task_id = self.task_id
        output.sample_index = self._sample_index
        for index, centerline in enumerate(centerlines):
            points_original = restore_points(centerline.points_xy, transform)
            projection = project_centerline_to_3d(
                points_original,
                depth,
                intrinsics,
                self.projection_config,
                mount_position_m=rail_position,
            )
            foreground = centerline.cleaned_mask > 0
            confidence = (
                float(probability[foreground].mean()) * projection.valid_ratio
                if np.any(foreground)
                else 0.0
            )
            candidate = WeldSeamCandidate()
            candidate.observation_id = f"{self.role}_{self._sample_index}_{index}"
            candidate.points = [
                Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                for point in projection.points_base_xyz_m
            ]
            candidate.confidence = float(np.clip(confidence, 0.0, 1.0))
            candidate.rail_position_at_capture_m = float(rail_position or 0.0)
            candidate.valid = (
                len(candidate.points) >= self.projection_config.probe_minimum_alignment_points
                and candidate.confidence >= self.minimum_confidence
            )
            candidate.detail = (
                f"valid_ratio={projection.valid_ratio:.3f};points={len(candidate.points)}"
            )
            output.candidates.append(candidate)
        self.candidate_publisher.publish(output)
        completed = time.perf_counter()
        processing_s = completed - started
        end_to_end_s = max(0.0, (self.get_clock().now().nanoseconds - capture_ns) / 1e9)
        self.latency.add(processing_s, end_to_end_s, completed)
        self._processed += 1
        self._sample_index += 1
        self._last_error = "ok" if output.candidates else "no_weld_candidates"
        self._publish_timing(output, processing_s, end_to_end_s)

    def _publish_timing(self, source, processing_s: float, end_to_end_s: float) -> None:
        snapshot = self.latency.snapshot()
        timing = VisionTiming()
        timing.header = source.header
        timing.camera_role = self.role
        timing.processing_ms = float(processing_s * 1000.0)
        timing.end_to_end_ms = float(end_to_end_s * 1000.0)
        timing.processing_p95_ms = float(snapshot.processing_p95_ms)
        timing.end_to_end_p95_ms = float(snapshot.capture_age_p95_ms)
        timing.effective_hz = float(snapshot.effective_hz)
        timing.processed_frames = self._processed
        timing.dropped_frames = self.frames.dropped + self._superseded
        timing.deadline_met = (
            processing_s <= self.maximum_processing_s
            and end_to_end_s <= self.maximum_capture_age_s
        )
        timing.detail = self._last_error
        self.timing_publisher.publish(timing)

    def _publish_state(self) -> None:
        state = DeviceState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self.projection_config.base_frame
        state.device_name = f"{self.role}_weld_vision"
        state.enabled = True
        state.ready = self._intrinsics is not None and self._worker.is_alive()
        state.acquiring = self._processed > 0
        state.data_valid = self._last_error in ("ok", "no_weld_candidates")
        state.fault = not self._worker.is_alive() or self._last_error.startswith("processing_error")
        state.detail = (
            f"{self._last_error};processed={self._processed};"
            f"dropped={self.frames.dropped + self._superseded}"
        )
        self.state_publisher.publish(state)

    def destroy_node(self):
        self._stop.set()
        self._wake.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RgbdWeldLocalizer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
