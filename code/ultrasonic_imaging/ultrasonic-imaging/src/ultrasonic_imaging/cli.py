from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path
from time import perf_counter

import numpy as np

from .beamforming import compound_pwi, tfm_fmc
from .io import load_paut_archive, load_tofd, save_tofd_npz, write_json
from .models import FmcData, ImageGrid, PwiData
from .plotting import save_depth_image, save_tofd_bscan
from .signal import preprocess_rf
from .simulation import simulate_tofd
from .tofd import (
    bscan,
    depth_corrected_scan,
    detect_tip_candidates,
    pair_tip_candidates,
    saft,
    select_confident_defects,
    suppress_common_mode,
)
from .tofd_evaluation import defect_to_dict, evaluate_dataset, tip_to_dict


def _mm_grid(minimum_mm: float, maximum_mm: float, step_mm: float) -> np.ndarray:
    if maximum_mm <= minimum_mm or step_mm <= 0:
        raise ValueError("grid maximum must exceed minimum and step must be positive")
    count = int(np.floor((maximum_mm - minimum_mm) / step_mm + 0.5)) + 1
    return np.linspace(minimum_mm, maximum_mm, count, dtype=np.float64) * 1e-3


def _band_hz(args: argparse.Namespace) -> tuple[float, float] | None:
    if args.band_low_mhz is None and args.band_high_mhz is None:
        return None
    if args.band_low_mhz is None or args.band_high_mhz is None:
        raise ValueError("both --band-low-mhz and --band-high-mhz are required")
    return args.band_low_mhz * 1e6, args.band_high_mhz * 1e6


def _peak(db: np.ndarray, x_m: np.ndarray, z_m: np.ndarray) -> dict[str, float]:
    z_index, x_index = np.unravel_index(int(np.argmax(db)), db.shape)
    return {
        "x_mm": float(x_m[x_index] * 1e3),
        "z_mm": float(z_m[z_index] * 1e3),
        "db": float(db[z_index, x_index]),
    }


def _save_beamform_result(
    output: Path,
    result: object,
    title: str,
    floor_db: float,
    report: dict[str, object],
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output / "image.npz",
        image=result.image,
        amplitude=result.amplitude,
        db=result.db,
        coherence_factor=(
            result.coherence_factor
            if result.coherence_factor is not None
            else np.empty((0,), dtype=np.float32)
        ),
        x_m=result.x_m,
        z_m=result.z_m,
    )
    save_depth_image(output / "image.png", result.db, result.x_m, result.z_m, title, floor_db)
    report["peak"] = _peak(result.db, result.x_m, result.z_m)
    report["output_files"] = ["image.npz", "image.png", "report.json"]
    write_json(output / "report.json", report)


def command_paut(args: argparse.Namespace) -> int:
    started = perf_counter()
    data = load_paut_archive(args.metadata)
    if args.velocity_m_s is not None:
        data = replace(data, velocity_m_s=args.velocity_m_s)
    filtered = preprocess_rf(
        data.rf,
        data.sample_rate_hz,
        band_hz=_band_hz(args),
        time_gain_db_per_s=args.time_gain_db_per_s,
    )
    data = replace(data, rf=filtered)
    grid = ImageGrid(
        x_m=_mm_grid(args.x_min_mm, args.x_max_mm, args.pixel_mm),
        z_m=_mm_grid(args.z_min_mm, args.z_max_mm, args.pixel_mm),
    )
    if isinstance(data, FmcData):
        element_z_m = None
        if args.element_coordinates is not None:
            with np.load(args.element_coordinates, allow_pickle=False) as coordinates:
                element_x_m = np.asarray(coordinates["element_x_m"], dtype=np.float64)
                element_z_m = np.asarray(coordinates["element_z_m"], dtype=np.float64)
            data = replace(data, element_x_m=element_x_m)
        if args.wedge_velocity_m_s is not None and element_z_m is None:
            raise ValueError(
                "--wedge-velocity-m-s requires --element-coordinates with element_x_m and element_z_m"
            )
        result = tfm_fmc(
            data,
            grid,
            use_coherence=args.coherence,
            floor_db=args.floor_db,
            max_tx_rx_separation=args.max_tx_rx_separation,
            element_z_m=element_z_m,
            wedge_velocity_m_s=args.wedge_velocity_m_s,
        )
        method = "FMC-TFM"
    elif isinstance(data, PwiData):
        if args.wedge_velocity_m_s is not None or args.element_coordinates is not None:
            raise ValueError("two-layer wedge correction is currently implemented for FMC-TFM only")
        result = compound_pwi(
            data,
            grid,
            use_coherence=args.coherence,
            floor_db=args.floor_db,
        )
        method = "PWI coherent compounding"
    else:  # pragma: no cover
        raise TypeError("unsupported phased-array data type")
    report: dict[str, object] = {
        "method": method,
        "dataset_id": data.metadata.get("dataset_id"),
        "input_metadata": str(Path(args.metadata).resolve()),
        "rf_shape": list(data.rf.shape),
        "sample_rate_hz": data.sample_rate_hz,
        "velocity_m_s": data.velocity_m_s,
        "center_frequency_hz": data.center_frequency_hz,
        "coherence_factor": args.coherence,
        "grid": {
            "x_min_mm": args.x_min_mm,
            "x_max_mm": args.x_max_mm,
            "z_min_mm": args.z_min_mm,
            "z_max_mm": args.z_max_mm,
            "pixel_mm": args.pixel_mm,
        },
        "processing_seconds": perf_counter() - started,
        "limitations": [
            "The result is an algorithm-development image, not a qualified inspection decision.",
            "Material velocity, acoustic origin and wedge geometry require specimen-specific calibration.",
        ],
    }
    _save_beamform_result(Path(args.output), result, method, args.floor_db, report)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


def command_tofd_simulate(args: argparse.Namespace) -> int:
    data = simulate_tofd(
        scan_start_m=args.scan_start_mm * 1e-3,
        scan_end_m=args.scan_end_mm * 1e-3,
        scan_step_m=args.scan_step_mm * 1e-3,
        sample_rate_hz=args.sample_rate_mhz * 1e6,
        sample_count=args.sample_count,
        velocity_m_s=args.velocity_m_s,
        probe_center_spacing_m=args.pcs_mm * 1e-3,
        plate_thickness_m=args.thickness_mm * 1e-3,
        center_frequency_hz=args.frequency_mhz * 1e6,
        flaw_scan_position_m=args.flaw_x_mm * 1e-3,
        upper_tip_depth_m=args.upper_tip_mm * 1e-3,
        lower_tip_depth_m=args.lower_tip_mm * 1e-3,
        noise_std=args.noise_std,
        seed=args.seed,
    )
    destination = save_tofd_npz(args.output, data)
    print(f"Synthetic TOFD data written to {destination}")
    print("This file is physics-based simulated RF, not measured inspection data.")
    return 0


def command_tofd_image(args: argparse.Namespace) -> int:
    started = perf_counter()
    data = load_tofd(args.input)
    if args.velocity_m_s is not None:
        data = replace(data, velocity_m_s=args.velocity_m_s)
    if args.pcs_mm is not None:
        data = replace(data, probe_center_spacing_m=args.pcs_mm * 1e-3)
    band_hz = _band_hz(args)
    if band_hz is None and data.center_frequency_hz is not None:
        band_hz = (
            max(0.5e6, 0.4 * data.center_frequency_hz),
            min(0.45 * data.sample_rate_hz, 1.7 * data.center_frequency_hz),
        )
    data = replace(
        data,
        rf=preprocess_rf(
            data.rf,
            data.sample_rate_hz,
            band_hz=band_hz,
            time_gain_db_per_s=args.time_gain_db_per_s,
        ),
    )
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    z_m = _mm_grid(args.z_min_mm, args.z_max_mm, args.pixel_mm)
    x_m = _mm_grid(args.x_min_mm, args.x_max_mm, args.pixel_mm)
    grid = ImageGrid(x_m=x_m, z_m=z_m)

    b_result = bscan(data, args.floor_db)
    d_result = depth_corrected_scan(data, z_m, args.floor_db)
    s_result = saft(
        suppress_common_mode(data),
        grid,
        aperture_m=None if args.aperture_mm is None else args.aperture_mm * 1e-3,
        use_coherence=args.coherence,
        floor_db=args.floor_db,
    )
    tip_candidates = detect_tip_candidates(
        s_result,
        threshold_db=args.tip_threshold_db,
        min_separation_m=max(1e-3, 4 * args.pixel_mm * 1e-3),
        max_candidates=60,
    )
    paired_defects = pair_tip_candidates(
        tip_candidates,
        max_defects=args.max_defects,
        max_x_difference_m=args.tip_pair_x_mm * 1e-3,
        min_height_m=args.min_height_mm * 1e-3,
        max_height_m=args.max_height_mm * 1e-3 if args.max_height_mm else None,
        min_phase_difference_rad=2.0,
    )
    sized_defects = select_confident_defects(s_result, paired_defects)
    np.savez_compressed(
        output / "tofd_images.npz",
        bscan_rf=b_result.rf,
        bscan_envelope=b_result.envelope,
        bscan_db=b_result.db,
        bscan_time_s=b_result.time_s,
        scan_positions_m=b_result.scan_positions_m,
        dscan_amplitude=d_result.amplitude,
        dscan_db=d_result.db,
        dscan_depth_m=d_result.depth_m,
        saft_image=s_result.image,
        saft_amplitude=s_result.amplitude,
        saft_db=s_result.db,
        saft_x_m=s_result.x_m,
        saft_z_m=s_result.z_m,
        coherence_factor=(
            s_result.coherence_factor
            if s_result.coherence_factor is not None
            else np.empty((0,), dtype=np.float32)
        ),
    )
    save_tofd_bscan(output / "bscan.png", b_result.rf, b_result.scan_positions_m, b_result.time_s)
    save_depth_image(
        output / "dscan_depth_corrected.png",
        d_result.db.T,
        d_result.scan_positions_m,
        d_result.depth_m,
        "TOFD depth-corrected D-scan",
        args.floor_db,
        xlabel="Scan position (mm)",
    )
    save_depth_image(
        output / "saft.png",
        s_result.db,
        s_result.x_m,
        s_result.z_m,
        "TOFD SAFT",
        args.floor_db,
        xlabel="Scan position (mm)",
    )
    report = {
        "method": "TOFD B-scan, depth correction and SAFT",
        "input": str(Path(args.input).resolve()),
        "input_is_measured": data.metadata.get("measured_data"),
        "rf_shape": list(data.rf.shape),
        "sample_rate_hz": data.sample_rate_hz,
        "velocity_m_s": data.velocity_m_s,
        "probe_center_spacing_m": data.probe_center_spacing_m,
        "plate_thickness_m": data.plate_thickness_m,
        "coherence_factor": args.coherence,
        "saft_peak": _peak(s_result.db, s_result.x_m, s_result.z_m),
        "tip_candidates": [tip_to_dict(item) for item in tip_candidates],
        "sized_defects": [defect_to_dict(item) for item in sized_defects],
        "processing_seconds": perf_counter() - started,
        "output_files": [
            "tofd_images.npz",
            "bscan.png",
            "dscan_depth_corrected.png",
            "saft.png",
            "report.json",
        ],
        "limitations": [
            "TOFD depth conversion assumes symmetric probes, known PCS and a homogeneous specimen.",
            "Wedge delays and lateral-wave time zero must be calibrated for measured data.",
            "Reported peaks are image maxima, not automatic defect acceptance decisions.",
        ],
    }
    write_json(output / "report.json", report)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


def command_tofd_evaluate(args: argparse.Namespace) -> int:
    summary = evaluate_dataset(
        args.dataset,
        args.output,
        split=args.split,
        pixel_m=args.pixel_mm * 1e-3,
        threshold_db=args.tip_threshold_db,
        limit=args.limit,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


def _add_grid_arguments(parser: argparse.ArgumentParser, *, x_min: float, x_max: float, z_max: float) -> None:
    parser.add_argument("--x-min-mm", type=float, default=x_min)
    parser.add_argument("--x-max-mm", type=float, default=x_max)
    parser.add_argument("--z-min-mm", type=float, default=1.0)
    parser.add_argument("--z-max-mm", type=float, default=z_max)
    parser.add_argument("--pixel-mm", type=float, default=0.25)
    parser.add_argument("--floor-db", type=float, default=-60.0)


def _add_processing_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--band-low-mhz", type=float)
    parser.add_argument("--band-high-mhz", type=float)
    parser.add_argument("--time-gain-db-per-s", type=float, default=0.0)
    parser.add_argument("--coherence", action="store_true")
    parser.add_argument("--velocity-m-s", type=float)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ultra-image",
        description="Offline phased-array FMC/PWI and TOFD ultrasonic imaging",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    paut = subparsers.add_parser("paut", help="form an FMC-TFM or PWI image")
    paut.add_argument("--metadata", required=True, help="archived dataset metadata.json")
    paut.add_argument("--output", required=True)
    _add_grid_arguments(paut, x_min=-25.0, x_max=25.0, z_max=35.0)
    _add_processing_arguments(paut)
    paut.add_argument("--max-tx-rx-separation", type=int)
    paut.add_argument("--element-coordinates", help="NPZ containing element_x_m and element_z_m")
    paut.add_argument("--wedge-velocity-m-s", type=float)
    paut.set_defaults(handler=command_paut)

    simulate = subparsers.add_parser("tofd-simulate", help="create a labelled synthetic TOFD RF dataset")
    simulate.add_argument("--output", required=True)
    simulate.add_argument("--scan-start-mm", type=float, default=-50.0)
    simulate.add_argument("--scan-end-mm", type=float, default=50.0)
    simulate.add_argument("--scan-step-mm", type=float, default=0.5)
    simulate.add_argument("--sample-rate-mhz", type=float, default=100.0)
    simulate.add_argument("--sample-count", type=int, default=4096)
    simulate.add_argument("--velocity-m-s", type=float, default=5890.0)
    simulate.add_argument("--pcs-mm", type=float, default=60.0)
    simulate.add_argument("--thickness-mm", type=float, default=30.0)
    simulate.add_argument("--frequency-mhz", type=float, default=5.0)
    simulate.add_argument("--flaw-x-mm", type=float, default=0.0)
    simulate.add_argument("--upper-tip-mm", type=float, default=12.0)
    simulate.add_argument("--lower-tip-mm", type=float, default=20.0)
    simulate.add_argument("--noise-std", type=float, default=0.035)
    simulate.add_argument("--seed", type=int, default=7)
    simulate.set_defaults(handler=command_tofd_simulate)

    tofd = subparsers.add_parser("tofd", help="form TOFD B-scan, D-scan and SAFT images")
    tofd.add_argument("--input", required=True, help="TOFD NPZ or archived MATLAB v7.3 MAT input")
    tofd.add_argument("--output", required=True)
    _add_grid_arguments(tofd, x_min=-50.0, x_max=50.0, z_max=30.0)
    _add_processing_arguments(tofd)
    tofd.add_argument("--pcs-mm", type=float, help="override probe center spacing")
    tofd.add_argument("--aperture-mm", type=float, default=40.0)
    tofd.add_argument("--tip-threshold-db", type=float, default=-36.0)
    tofd.add_argument("--tip-pair-x-mm", type=float, default=1.5)
    tofd.add_argument("--min-height-mm", type=float, default=1.5)
    tofd.add_argument("--max-height-mm", type=float)
    tofd.add_argument("--max-defects", type=int, default=2)
    tofd.set_defaults(handler=command_tofd_image)

    evaluate = subparsers.add_parser(
        "tofd-evaluate", help="benchmark automatic TOFD tip localization and sizing"
    )
    evaluate.add_argument("--dataset", required=True, help="dataset root containing manifest.csv")
    evaluate.add_argument("--output", required=True)
    evaluate.add_argument(
        "--split", choices=["train", "validation", "test", "all"], default="test"
    )
    evaluate.add_argument("--pixel-mm", type=float, default=0.25)
    evaluate.add_argument("--tip-threshold-db", type=float, default=-36.0)
    evaluate.add_argument("--limit", type=int)
    evaluate.set_defaults(handler=command_tofd_evaluate)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.handler(args))
    except (FileNotFoundError, KeyError, ValueError, TypeError) as exc:
        parser.error(str(exc))
        return 2


if __name__ == "__main__":
    sys.exit(main())
