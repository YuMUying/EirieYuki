from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def save_depth_image(
    path: str | Path,
    db: np.ndarray,
    x_m: np.ndarray,
    z_m: np.ndarray,
    title: str,
    floor_db: float = -60.0,
    xlabel: str = "Lateral position (mm)",
) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 5.6), constrained_layout=True)
    image = ax.imshow(
        db,
        extent=[x_m[0] * 1e3, x_m[-1] * 1e3, z_m[-1] * 1e3, z_m[0] * 1e3],
        aspect="auto",
        cmap="inferno",
        vmin=floor_db,
        vmax=0.0,
        interpolation="nearest",
    )
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Depth (mm)")
    fig.colorbar(image, ax=ax, label="Normalized amplitude (dB)")
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path


def save_tofd_bscan(
    path: str | Path,
    rf: np.ndarray,
    scan_m: np.ndarray,
    time_s: np.ndarray,
    title: str = "TOFD B-scan",
) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    limit = float(np.percentile(np.abs(rf), 99.5)) or 1.0
    fig, ax = plt.subplots(figsize=(8.4, 5.6), constrained_layout=True)
    image = ax.imshow(
        rf.T,
        extent=[scan_m[0] * 1e3, scan_m[-1] * 1e3, time_s[-1] * 1e6, time_s[0] * 1e6],
        aspect="auto",
        cmap="seismic",
        vmin=-limit,
        vmax=limit,
        interpolation="nearest",
    )
    ax.set_title(title)
    ax.set_xlabel("Scan position (mm)")
    ax.set_ylabel("Time of flight (us)")
    fig.colorbar(image, ax=ax, label="RF amplitude")
    fig.savefig(path, dpi=170)
    plt.close(fig)
    return path
