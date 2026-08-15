# Ultrasonic Imaging

Offline FMC/PWI phased-array and TOFD imaging tools for the Robot Challenge Cup project.

## Install and test

From the version.1 project root:

```powershell
cd code\ultrasonic_imaging\ultrasonic-imaging
python -m pip install -e ".[test]"
python -m pytest
```

Python 3.10 or newer is required. MATLAB v7.3 TOFD files require `h5py`, included in the normal dependencies.

## Project data locations

The active archive is outside the code package:

```text
datasets/ultrasonic/Ultrasonic_Weld_Imaging
```

The synthetic TOFD dataset used by the evaluator is:

```text
datasets/ultrasonic/Ultrasonic_Weld_Imaging/derived/tofd_synthetic_dataset_v1
```

Paths passed to the CLI may be absolute or relative to the current directory. The following commands assume the current directory is `code/ultrasonic_imaging/ultrasonic-imaging`.

## TOFD archived MAT input

Image one archived case:

```powershell
$dataset = "..\..\..\datasets\ultrasonic\Ultrasonic_Weld_Imaging\derived\tofd_synthetic_dataset_v1"
$output = "..\..\..\results\ultrasonic_imaging"
ultra-image tofd `
  --input "$dataset\test\case_0110\tofd_synthetic_scan.mat" `
  --output "$output\tofd_case_0110" `
  --x-min-mm -18 --x-max-mm 18 `
  --z-min-mm 1.5 --z-max-mm 34 `
  --pixel-mm 0.25 --aperture-mm 40
```

The command writes `tofd_images.npz`, three PNG images, and `report.json`. `tip_candidates` are local SAFT maxima. `sized_defects` require compatible upper and lower candidates and are reported separately.

Evaluate a complete split:

```powershell
ultra-image tofd-evaluate `
  --dataset $dataset `
  --split test `
  --output "$output\tofd_evaluation"
```

The evaluator writes `summary.json`, `cases.json`, and `cases.csv`, including tip recall, sizing recall, localization/height errors, and normal-case false-positive rate.

## Other commands

- `ultra-image paut`: FMC-TFM or PWI coherent compounding from archived metadata.
- `ultra-image tofd-simulate`: generate a lightweight NPZ TOFD demonstration.
- `ultra-image tofd`: image NPZ or archived MATLAB v7.3 MAT data.
- `ultra-image tofd-evaluate`: benchmark localization and sizing against archive truth.

Run `ultra-image <command> --help` for all options.

## Limits

The archived TOFD RF is semi-analytical synthetic research data. Its amplitudes, clutter, beam model, and electronic noise do not establish field probability of detection. Material velocity, probe center spacing, wedge delay, acoustic origin, coupling, and sensitivity must be calibrated on the real probe, wedge, material batch, and reference block. Automatic indications are algorithm candidates, not acceptance-code decisions.
