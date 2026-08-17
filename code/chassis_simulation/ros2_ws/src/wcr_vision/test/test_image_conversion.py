from __future__ import annotations

import numpy as np
import pytest
from sensor_msgs.msg import Image

from wcr_vision.image_conversion import image_to_array


def message(array: np.ndarray, encoding: str) -> Image:
    channels = array.shape[2] if array.ndim == 3 else 1
    return Image(
        height=array.shape[0],
        width=array.shape[1],
        encoding=encoding,
        is_bigendian=0,
        step=array.shape[1] * channels * array.dtype.itemsize,
        data=array.tobytes(),
    )


def test_bgr8_and_16uc1_round_trip() -> None:
    color = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    depth = np.arange(8, dtype=np.uint16).reshape(2, 4)
    np.testing.assert_array_equal(image_to_array(message(color, "bgr8")), color)
    np.testing.assert_array_equal(image_to_array(message(depth, "16UC1")), depth)


def test_rgb_is_converted_to_bgr() -> None:
    rgb = np.array([[[1, 2, 3]]], dtype=np.uint8)
    np.testing.assert_array_equal(
        image_to_array(message(rgb, "rgb8")), np.array([[[3, 2, 1]]], dtype=np.uint8)
    )


def test_unsupported_encoding_is_rejected() -> None:
    mono = np.zeros((2, 2), dtype=np.uint8)
    with pytest.raises(ValueError, match="unsupported image encoding"):
        image_to_array(message(mono, "mono8"))
