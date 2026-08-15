from __future__ import annotations

import heapq
from dataclasses import dataclass

import cv2
import numpy as np


_NEIGHBORS = (
    (-1, -1, 2**0.5),
    (-1, 0, 1.0),
    (-1, 1, 2**0.5),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (1, -1, 2**0.5),
    (1, 0, 1.0),
    (1, 1, 2**0.5),
)


@dataclass(frozen=True)
class CenterlineResult:
    points_xy: np.ndarray
    cleaned_mask: np.ndarray
    skeleton: np.ndarray
    length_pixels: float


def clean_weld_mask(
    mask: np.ndarray,
    min_area: int = 100,
    close_kernel: int = 7,
    keep_largest: bool = True,
) -> np.ndarray:
    """Remove small regions and close narrow gaps in a binary weld mask."""
    binary = np.where(mask > 0, 255, 0).astype(np.uint8)
    if close_kernel > 1:
        close_kernel += 1 - close_kernel % 2
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_kernel, close_kernel)
        )
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return np.zeros_like(binary)
    areas = stats[1:, cv2.CC_STAT_AREA]
    valid = np.flatnonzero(areas >= max(1, min_area)) + 1
    if valid.size == 0:
        return np.zeros_like(binary)
    if keep_largest:
        valid = np.array([valid[np.argmax(stats[valid, cv2.CC_STAT_AREA])]])
    return np.where(np.isin(labels, valid), 255, 0).astype(np.uint8)


def _transitions(neighbors: list[np.ndarray]) -> np.ndarray:
    return sum(
        ((neighbors[index] == 0) & (neighbors[(index + 1) % 8] == 1))
        for index in range(8)
    )


def skeletonize(binary_mask: np.ndarray, max_iterations: int = 1000) -> np.ndarray:
    """Thin a mask with OpenCV Contrib, with a portable NumPy fallback."""
    binary = np.where(binary_mask > 0, 255, 0).astype(np.uint8)
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        return cv2.ximgproc.thinning(binary, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
    image = (binary > 0).astype(np.uint8)
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0

    for _ in range(max_iterations):
        changed = False
        for phase in (0, 1):
            padded = np.pad(image, 1, mode="constant")
            p2 = padded[:-2, 1:-1]
            p3 = padded[:-2, 2:]
            p4 = padded[1:-1, 2:]
            p5 = padded[2:, 2:]
            p6 = padded[2:, 1:-1]
            p7 = padded[2:, :-2]
            p8 = padded[1:-1, :-2]
            p9 = padded[:-2, :-2]
            neighbors = [p2, p3, p4, p5, p6, p7, p8, p9]
            neighbor_count = sum(neighbors)
            transitions = _transitions(neighbors)
            if phase == 0:
                condition_a = p2 * p4 * p6 == 0
                condition_b = p4 * p6 * p8 == 0
            else:
                condition_a = p2 * p4 * p8 == 0
                condition_b = p2 * p6 * p8 == 0
            remove = (
                (image == 1)
                & (neighbor_count >= 2)
                & (neighbor_count <= 6)
                & (transitions == 1)
                & condition_a
                & condition_b
            )
            if np.any(remove):
                image[remove] = 0
                changed = True
        if not changed:
            break
    return (image * 255).astype(np.uint8)


def _largest_skeleton_component(skeleton: np.ndarray) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(skeleton, connectivity=8)
    if count <= 1:
        return np.zeros_like(skeleton)
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return np.where(labels == largest, 255, 0).astype(np.uint8)


def _build_graph(skeleton: np.ndarray) -> tuple[np.ndarray, list[list[tuple[int, float]]]]:
    coordinates_yx = np.argwhere(skeleton > 0)
    index_by_pixel = {tuple(pixel): index for index, pixel in enumerate(coordinates_yx)}
    adjacency: list[list[tuple[int, float]]] = [[] for _ in coordinates_yx]
    for index, (y, x) in enumerate(coordinates_yx):
        for dy, dx, weight in _NEIGHBORS:
            neighbor = index_by_pixel.get((int(y + dy), int(x + dx)))
            if neighbor is not None:
                adjacency[index].append((neighbor, weight))
    return coordinates_yx, adjacency


def _dijkstra(
    adjacency: list[list[tuple[int, float]]], source: int
) -> tuple[np.ndarray, np.ndarray]:
    distances = np.full(len(adjacency), np.inf, dtype=np.float64)
    previous = np.full(len(adjacency), -1, dtype=np.int32)
    distances[source] = 0.0
    queue: list[tuple[float, int]] = [(0.0, source)]
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances[node]:
            continue
        for neighbor, weight in adjacency[node]:
            candidate = distance + weight
            if candidate < distances[neighbor]:
                distances[neighbor] = candidate
                previous[neighbor] = node
                heapq.heappush(queue, (candidate, neighbor))
    return distances, previous


def _farthest_finite(distances: np.ndarray) -> int:
    finite = np.flatnonzero(np.isfinite(distances))
    if finite.size == 0:
        return 0
    return int(finite[np.argmax(distances[finite])])


def _principal_path(skeleton: np.ndarray) -> tuple[np.ndarray, float]:
    coordinates_yx, adjacency = _build_graph(skeleton)
    if len(coordinates_yx) < 2:
        return coordinates_yx[:, ::-1].astype(np.float32), 0.0

    endpoints = [index for index, edges in enumerate(adjacency) if len(edges) == 1]
    source = endpoints[0] if endpoints else 0
    distances, _ = _dijkstra(adjacency, source)
    first = _farthest_finite(distances)
    distances, previous = _dijkstra(adjacency, first)
    second = _farthest_finite(distances)

    path_indices = [second]
    while path_indices[-1] != first:
        parent = int(previous[path_indices[-1]])
        if parent < 0:
            break
        path_indices.append(parent)
    path_indices.reverse()
    path_xy = coordinates_yx[path_indices, ::-1].astype(np.float32)
    return path_xy, float(distances[second])


def _endpoint_paths(
    skeleton: np.ndarray,
    minimum_straightness: float,
    maximum_endpoints: int = 12,
) -> list[tuple[np.ndarray, float, float]]:
    coordinates_yx, adjacency = _build_graph(skeleton)
    endpoints = [index for index, edges in enumerate(adjacency) if len(edges) == 1]
    if len(endpoints) < 3 or len(endpoints) > maximum_endpoints:
        points, length = _principal_path(skeleton)
        return [(points, length, 1.0)] if len(points) >= 2 else []
    paths: list[tuple[np.ndarray, float, float]] = []
    for source_offset, source in enumerate(endpoints[:-1]):
        distances, previous = _dijkstra(adjacency, source)
        for target in endpoints[source_offset + 1 : ]:
            length = float(distances[target])
            if not np.isfinite(length) or length <= 0.0:
                continue
            path_indices = [target]
            while path_indices[-1] != source:
                parent = int(previous[path_indices[-1]])
                if parent < 0:
                    path_indices = []
                    break
                path_indices.append(parent)
            if not path_indices:
                continue
            path_indices.reverse()
            points = coordinates_yx[path_indices, ::-1].astype(np.float32)
            chord = float(np.linalg.norm(points[-1] - points[0]))
            straightness = chord / length
            if straightness >= minimum_straightness:
                paths.append((points, length, straightness))
    if paths:
        paths.sort(key=lambda item: (item[2], item[1]), reverse=True)
        return paths
    points, length = _principal_path(skeleton)
    return [(points, length, 1.0)] if len(points) >= 2 else []


def _orient_path(points_xy: np.ndarray, direction: str) -> np.ndarray:
    if len(points_xy) < 2:
        return points_xy
    if direction == "auto":
        span = np.ptp(points_xy, axis=0)
        axis = int(np.argmax(span))
        reverse = points_xy[0, axis] > points_xy[-1, axis]
    elif direction == "left-to-right":
        reverse = points_xy[0, 0] > points_xy[-1, 0]
    elif direction == "top-to-bottom":
        reverse = points_xy[0, 1] > points_xy[-1, 1]
    else:
        raise ValueError(f"Unknown centerline direction: {direction}")
    return points_xy[::-1].copy() if reverse else points_xy


def _smooth_path(points_xy: np.ndarray, window: int) -> np.ndarray:
    if window < 3 or len(points_xy) < window:
        return points_xy
    window += 1 - window % 2
    radius = window // 2
    padded = np.pad(points_xy, ((radius, radius), (0, 0)), mode="edge")
    kernel = np.full(window, 1.0 / window, dtype=np.float32)
    smooth = np.column_stack(
        [np.convolve(padded[:, axis], kernel, mode="valid") for axis in range(2)]
    ).astype(np.float32)
    smooth[0] = points_xy[0]
    smooth[-1] = points_xy[-1]
    return smooth


def resample_path(points_xy: np.ndarray, spacing: float) -> np.ndarray:
    if len(points_xy) < 2 or spacing <= 0:
        return points_xy.astype(np.float32)
    segment_lengths = np.linalg.norm(np.diff(points_xy, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total = float(cumulative[-1])
    if total <= 0:
        return points_xy[:1].astype(np.float32)
    count = max(2, int(np.ceil(total / spacing)) + 1)
    samples = np.linspace(0.0, total, count)
    return np.column_stack(
        [np.interp(samples, cumulative, points_xy[:, axis]) for axis in range(2)]
    ).astype(np.float32)


def extract_centerline(
    mask: np.ndarray,
    min_area: int = 100,
    close_kernel: int = 7,
    point_spacing: float = 8.0,
    smooth_window: int = 7,
    direction: str = "auto",
) -> CenterlineResult:
    cleaned = clean_weld_mask(mask, min_area, close_kernel, keep_largest=True)
    skeleton = _largest_skeleton_component(skeletonize(cleaned))
    points_xy, length_pixels = _principal_path(skeleton)
    points_xy = _orient_path(points_xy, direction)
    points_xy = _smooth_path(points_xy, smooth_window)
    points_xy = resample_path(points_xy, point_spacing)
    return CenterlineResult(points_xy, cleaned, skeleton, length_pixels)


def extract_centerlines(
    mask: np.ndarray,
    min_area: int = 100,
    close_kernel: int = 7,
    point_spacing: float = 8.0,
    smooth_window: int = 7,
    direction: str = "auto",
    minimum_separation_pixels: float = 12.0,
    minimum_branch_straightness: float = 0.82,
    maximum_centerlines: int = 8,
) -> list[CenterlineResult]:
    """Extract spatially distinct centerlines from all valid weld regions."""
    if minimum_separation_pixels <= 0:
        raise ValueError("minimum centerline separation must be positive")
    if not 0.0 < minimum_branch_straightness <= 1.0:
        raise ValueError("branch straightness must be in (0, 1]")
    if maximum_centerlines < 1:
        raise ValueError("maximum centerline count must be positive")
    cleaned_all = clean_weld_mask(
        mask, min_area=min_area, close_kernel=close_kernel, keep_largest=False
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        cleaned_all, connectivity=8
    )
    component_ids = list(range(1, count))
    component_ids.sort(
        key=lambda label: int(stats[label, cv2.CC_STAT_AREA]), reverse=True
    )
    results: list[CenterlineResult] = []
    for label in component_ids:
        component = np.where(labels == label, 255, 0).astype(np.uint8)
        skeleton = _largest_skeleton_component(skeletonize(component))
        for points_xy, length_pixels, _ in _endpoint_paths(
            skeleton, minimum_branch_straightness
        ):
            points_xy = _orient_path(points_xy, direction)
            points_xy = _smooth_path(points_xy, smooth_window)
            points_xy = resample_path(points_xy, point_spacing)
            if len(points_xy) < 2:
                continue
            is_distinct = True
            for existing in results:
                deltas = points_xy[:, None, :] - existing.points_xy[None, :, :]
                distances = np.linalg.norm(deltas, axis=2)
                symmetric_distance = 0.5 * (
                    float(np.mean(np.min(distances, axis=1)))
                    + float(np.mean(np.min(distances, axis=0)))
                )
                if symmetric_distance < minimum_separation_pixels:
                    is_distinct = False
                    break
            if is_distinct:
                results.append(
                    CenterlineResult(points_xy, component, skeleton, length_pixels)
                )
            if len(results) >= maximum_centerlines:
                return results
    return results
