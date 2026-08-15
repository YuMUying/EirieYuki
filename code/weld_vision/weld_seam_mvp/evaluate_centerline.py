from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import torch

from weld_seam.centerline import extract_centerline
from weld_seam.checkpoint import load_checkpoint
from weld_seam.dataset import paired_samples
from project_paths import DEFAULT_CHECKPOINT, WELD_DATASET, WELD_RESULTS_DIR


def nearest_distances(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    differences = first[:, None, :] - second[None, :, :]
    distances = np.sqrt(np.sum(differences * differences, axis=2))
    return np.min(distances, axis=1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Evaluate centerlines extracted from predicted and ground-truth regions."
    )
    parser.add_argument("--data", type=Path, default=WELD_DATASET)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--split", choices=("val", "test"), default="test")
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--point-spacing", type=float, default=4.0)
    parser.add_argument("--min-area", type=int, default=100)
    parser.add_argument(
        "--output",
        type=Path,
        default=WELD_RESULTS_DIR / "evaluation" / "centerline_test_metrics.json",
    )
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model, checkpoint = load_checkpoint(args.checkpoint, device)
    image_size = int(checkpoint["image_size"])
    pairs = paired_samples(args.data, args.split)

    per_image: list[dict[str, float | str]] = []
    all_distances: list[np.ndarray] = []
    missing_predictions = 0
    for index, (image_path, mask_path) in enumerate(pairs, start=1):
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        truth_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if image is None or truth_mask is None:
            raise RuntimeError(f"Failed to read pair: {image_path}, {mask_path}")
        original_height, original_width = truth_mask.shape
        resized = cv2.resize(image, (image_size, image_size), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(np.ascontiguousarray(rgb.transpose(2, 0, 1)))
        tensor = tensor.float().div_(255.0).unsqueeze(0).to(device)
        with torch.inference_mode():
            probability = torch.sigmoid(model(tensor))[0, 0].cpu().numpy()
        probability = cv2.resize(
            probability,
            (original_width, original_height),
            interpolation=cv2.INTER_LINEAR,
        )
        predicted_mask = np.where(probability >= args.threshold, 255, 0).astype(np.uint8)
        predicted = extract_centerline(
            predicted_mask,
            min_area=args.min_area,
            point_spacing=args.point_spacing,
        )
        truth = extract_centerline(
            truth_mask,
            min_area=args.min_area,
            point_spacing=args.point_spacing,
        )
        if len(predicted.points_xy) < 2 or len(truth.points_xy) < 2:
            missing_predictions += int(len(predicted.points_xy) < 2)
            continue

        predicted_to_truth = nearest_distances(predicted.points_xy, truth.points_xy)
        truth_to_predicted = nearest_distances(truth.points_xy, predicted.points_xy)
        symmetric = np.concatenate((predicted_to_truth, truth_to_predicted))
        all_distances.append(symmetric)
        direct_endpoints = (
            np.linalg.norm(predicted.points_xy[0] - truth.points_xy[0])
            + np.linalg.norm(predicted.points_xy[-1] - truth.points_xy[-1])
        ) / 2.0
        reverse_endpoints = (
            np.linalg.norm(predicted.points_xy[0] - truth.points_xy[-1])
            + np.linalg.norm(predicted.points_xy[-1] - truth.points_xy[0])
        ) / 2.0
        per_image.append(
            {
                "sample": image_path.stem,
                "mean_symmetric_distance_px": float(symmetric.mean()),
                "p95_symmetric_distance_px": float(np.quantile(symmetric, 0.95)),
                "endpoint_error_px": float(min(direct_endpoints, reverse_endpoints)),
                "coverage_within_5px": float(np.mean(symmetric <= 5.0)),
                "predicted_length_px": predicted.length_pixels,
                "truth_length_px": truth.length_pixels,
            }
        )
        if index % 50 == 0:
            print(f"processed={index}/{len(pairs)}")

    if not all_distances:
        raise RuntimeError("No valid centerline pairs were produced")
    distances = np.concatenate(all_distances)
    endpoint_errors = np.asarray([row["endpoint_error_px"] for row in per_image])
    result = {
        "split": args.split,
        "samples": len(pairs),
        "valid_centerline_pairs": len(per_image),
        "missing_predictions": missing_predictions,
        "coordinate_resolution": "original_image_pixels",
        "threshold": args.threshold,
        "mean_symmetric_distance_px": float(distances.mean()),
        "median_symmetric_distance_px": float(np.median(distances)),
        "p95_symmetric_distance_px": float(np.quantile(distances, 0.95)),
        "mean_endpoint_error_px": float(endpoint_errors.mean()),
        "coverage_within_3px": float(np.mean(distances <= 3.0)),
        "coverage_within_5px": float(np.mean(distances <= 5.0)),
        "per_image": per_image,
    }
    text = json.dumps(result, indent=2)
    print(json.dumps({key: value for key, value in result.items() if key != "per_image"}, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
