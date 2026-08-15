from __future__ import annotations

import random
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def paired_samples(data_root: Path, split: str, original_only: bool = False) -> list[tuple[Path, Path]]:
    split_root = data_root / "supervised" / split
    image_dir = split_root / "images"
    mask_dir = split_root / "masks"
    if not image_dir.is_dir() or not mask_dir.is_dir():
        raise FileNotFoundError(f"Expected images and masks below: {split_root}")

    images = {
        path.stem: path
        for path in image_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    }
    masks = {
        path.stem: path
        for path in mask_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    }
    names = sorted(images.keys() & masks.keys())
    if original_only:
        names = [name for name in names if "__aug" not in name]
    if not names:
        raise RuntimeError(f"No paired samples found below: {split_root}")
    if images.keys() != masks.keys():
        missing_masks = sorted(images.keys() - masks.keys())[:5]
        missing_images = sorted(masks.keys() - images.keys())[:5]
        raise RuntimeError(
            f"Unpaired data in {split}: missing masks={missing_masks}, "
            f"missing images={missing_images}"
        )
    return [(images[name], masks[name]) for name in names]


def _augment(image: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if random.random() < 0.5:
        image = cv2.flip(image, 1)
        mask = cv2.flip(mask, 1)
    if random.random() < 0.2:
        image = cv2.flip(image, 0)
        mask = cv2.flip(mask, 0)
    if random.random() < 0.7:
        alpha = random.uniform(0.80, 1.20)
        beta = random.uniform(-20.0, 20.0)
        image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    if random.random() < 0.25:
        kernel = random.choice((3, 5))
        image = cv2.GaussianBlur(image, (kernel, kernel), 0)
    return image, mask


class WeldSegmentationDataset(Dataset):
    def __init__(
        self,
        data_root: str | Path,
        split: str,
        image_size: int = 192,
        augment: bool = False,
        original_only: bool = False,
        transform: Callable | None = None,
    ) -> None:
        self.samples = paired_samples(Path(data_root), split, original_only)
        self.image_size = image_size
        self.augment = augment
        self.transform = transform

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor, str]:
        image_path, mask_path = self.samples[index]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if image is None or mask is None:
            raise RuntimeError(f"Failed to read pair: {image_path}, {mask_path}")

        size = (self.image_size, self.image_size)
        image = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
        mask = cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST)
        if self.augment:
            image, mask = _augment(image, mask)
        if self.transform is not None:
            image, mask = self.transform(image, mask)

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_tensor = torch.from_numpy(np.ascontiguousarray(image.transpose(2, 0, 1)))
        image_tensor = image_tensor.float().div_(255.0)
        mask_tensor = torch.from_numpy(np.ascontiguousarray(mask[None] > 127)).float()
        return image_tensor, mask_tensor, image_path.stem
