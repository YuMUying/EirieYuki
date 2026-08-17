from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from .preprocess import (
    LetterboxTransform,
    crop_probability,
    letterbox,
)


class TorchSegmenter:
    def __init__(self, checkpoint_path: str | Path, device_name: str = "auto") -> None:
        import torch

        from .checkpoint import load_checkpoint

        if device_name == "auto":
            device_name = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device_name)
        self.model, checkpoint = load_checkpoint(Path(checkpoint_path), self.device)
        self.input_size = int(checkpoint["image_size"])

    def predict_resized(
        self, image_bgr: np.ndarray
    ) -> tuple[np.ndarray, LetterboxTransform]:
        import torch

        prepared, transform = letterbox(image_bgr, self.input_size)
        rgb = cv2.cvtColor(prepared, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)))
        tensor = tensor.float().div_(255.0).unsqueeze(0).to(self.device)
        with torch.inference_mode():
            probability = torch.sigmoid(self.model(tensor))[0, 0].cpu().numpy()
        return crop_probability(probability, transform), transform

    def predict(self, image_bgr: np.ndarray) -> np.ndarray:
        probability, transform = self.predict_resized(image_bgr)
        return cv2.resize(
            probability,
            (transform.original_width, transform.original_height),
            interpolation=cv2.INTER_LINEAR,
        )


class OnnxSegmenter:
    def __init__(
        self,
        model_path: str | Path,
        providers: list[str] | None = None,
        intra_op_threads: int = 0,
        inter_op_threads: int = 0,
    ) -> None:
        import onnxruntime as ort

        options = ort.SessionOptions()
        if intra_op_threads > 0:
            options.intra_op_num_threads = intra_op_threads
        if inter_op_threads > 0:
            options.inter_op_num_threads = inter_op_threads
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        selected = providers or ["CPUExecutionProvider"]
        available = set(ort.get_available_providers())
        missing = [provider for provider in selected if provider not in available]
        if missing:
            raise RuntimeError(
                f"Unavailable ONNX Runtime providers: {missing}; available={sorted(available)}"
            )
        self.session = ort.InferenceSession(
            str(model_path), sess_options=options, providers=selected
        )
        self.input_name = self.session.get_inputs()[0].name
        shape = self.session.get_inputs()[0].shape
        self.input_size = int(shape[-1])

    def predict_resized(
        self, image_bgr: np.ndarray
    ) -> tuple[np.ndarray, LetterboxTransform]:
        prepared, transform = letterbox(image_bgr, self.input_size)
        rgb = cv2.cvtColor(prepared, cv2.COLOR_BGR2RGB)
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32)
        tensor = tensor[None] / 255.0
        logits = self.session.run(None, {self.input_name: tensor})[0][0, 0]
        probability = 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))
        return crop_probability(probability, transform), transform

    def predict(self, image_bgr: np.ndarray) -> np.ndarray:
        probability, transform = self.predict_resized(image_bgr)
        return cv2.resize(
            probability,
            (transform.original_width, transform.original_height),
            interpolation=cv2.INTER_LINEAR,
        )
