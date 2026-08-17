from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class LetterboxTransform:
    original_width: int
    original_height: int
    input_size: int
    resized_width: int
    resized_height: int
    pad_left: int
    pad_top: int


def letterbox(image: np.ndarray, input_size: int) -> tuple[np.ndarray, LetterboxTransform]:
    height, width = image.shape[:2]
    scale = min(input_size / width, input_size / height)
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_AREA)
    pad_left = (input_size - resized_width) // 2
    pad_top = (input_size - resized_height) // 2
    output = np.zeros((input_size, input_size, 3), dtype=np.uint8)
    output[pad_top : pad_top + resized_height, pad_left : pad_left + resized_width] = resized
    transform = LetterboxTransform(
        width,
        height,
        input_size,
        resized_width,
        resized_height,
        pad_left,
        pad_top,
    )
    return output, transform


def restore_probability(probability: np.ndarray, transform: LetterboxTransform) -> np.ndarray:
    cropped = crop_probability(probability, transform)
    return cv2.resize(
        cropped,
        (transform.original_width, transform.original_height),
        interpolation=cv2.INTER_LINEAR,
    )


def crop_probability(
    probability: np.ndarray, transform: LetterboxTransform
) -> np.ndarray:
    return probability[
        transform.pad_top : transform.pad_top + transform.resized_height,
        transform.pad_left : transform.pad_left + transform.resized_width,
    ]


def restore_points(
    points_xy: np.ndarray, transform: LetterboxTransform
) -> np.ndarray:
    points = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
    if len(points) == 0:
        return points
    restored = np.empty_like(points)
    restored[:, 0] = (
        (points[:, 0] + 0.5) * transform.original_width / transform.resized_width
        - 0.5
    )
    restored[:, 1] = (
        (points[:, 1] + 0.5) * transform.original_height / transform.resized_height
        - 0.5
    )
    restored[:, 0] = np.clip(restored[:, 0], 0, transform.original_width - 1)
    restored[:, 1] = np.clip(restored[:, 1], 0, transform.original_height - 1)
    return restored
