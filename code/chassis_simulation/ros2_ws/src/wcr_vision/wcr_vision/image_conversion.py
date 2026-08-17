from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image


def image_to_array(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    if message.is_bigendian:
        raise ValueError("big-endian image messages are not supported")
    if encoding in ("bgr8", "rgb8"):
        channels, dtype = 3, np.uint8
    elif encoding in ("16uc1", "mono16"):
        channels, dtype = 1, np.uint16
    elif encoding == "32fc1":
        channels, dtype = 1, np.float32
    else:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    row_elements = int(message.step) // np.dtype(dtype).itemsize
    expected = int(message.height) * row_elements
    flat = np.frombuffer(message.data, dtype=dtype, count=expected)
    rows = flat.reshape(int(message.height), row_elements)
    width_elements = int(message.width) * channels
    image = rows[:, :width_elements]
    if channels > 1:
        image = image.reshape(int(message.height), int(message.width), channels)
        if encoding == "rgb8":
            image = image[:, :, ::-1]
    else:
        image = image.reshape(int(message.height), int(message.width))
    return np.ascontiguousarray(image)
