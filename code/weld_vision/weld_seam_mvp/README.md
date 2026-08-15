# Weld Seam Segmentation and RGB-D MVP

This component trains a compact binary weld-area segmentation model, extracts an ordered weld centerline, and projects it into camera and robot surface coordinates with aligned RGB-D data.

## Project locations

Paths are resolved by `project_paths.py` from the version.1 project root:

```text
code/weld_vision/weld_seam_mvp                       this code
datasets/weld_vision/WES-Combined-Dataset            active dataset
models/weld_vision/segmentation                       delivered models
results/weld_vision/training                          training history
results/weld_vision/inference                         inference output
results/weld_vision/evaluation                        evaluation output
```

The ready-to-use model files are:

- `models/weld_vision/segmentation/weld_segmentation.onnx`
- `models/weld_vision/segmentation/weld_segmentation.pt`
- `models/weld_vision/segmentation/model_card.json`

Independent test results on 223 images:

- Dice: `0.9867`
- IoU: `0.9737`
- Pixel accuracy: `0.9946`
- Valid centerlines: `223/223`
- Mean symmetric centerline error: `3.32 px`
- Median symmetric centerline error: `1.72 px`
- Centerline samples within 5 px: `92.10%`

Detailed historical metrics are under `results/weld_vision/training/unet_cpu_mvp`.

## Environment

Run from this component directory in WSL or Linux:

```bash
bash setup_wsl.sh
source .venv/bin/activate
```

The virtual environment is intentionally excluded from the organized project and is rebuilt locally by `setup_wsl.sh`.

## 2D inference

The model and output directory have project-relative defaults, so only the input is required:

```bash
python infer.py --input /path/to/camera_frame.png
```

A flat image directory is also accepted. Override defaults when needed:

```bash
python infer.py \
  --model /path/to/model.onnx \
  --input /path/to/images \
  --output /path/to/output
```

Each image produces probability, mask, edge, skeleton, overlay, centerline JSON, and centerline CSV outputs. The 2D frame uses image top-left as origin, with `x` right and `y` down.

## RGB-D camera and 3D curve

The selected camera is the RealSense D405. See `RGBD_D405_DESIGN.md` for field-of-view calculations, mounting, transforms, and calibration requirements. The supplied `config/d405_mount.yaml` is a mechanical design transform and must be replaced after physical camera-to-robot calibration.

The probe camera is a second D405 on the transverse rail. It uses `config/probe_d405_mount.yaml`; the two devices must have different serial-number, intrinsics, depth-scale and extrinsic records.

Capture one aligned frame:

```bash
pip install -r requirements-realsense.txt
python capture_realsense_rgbd.py --output captures/frame_0001
```

With two cameras, bind each role by serial number. Probe role selects `640x480@30` and the dynamic mount profile by default:

```bash
python capture_realsense_rgbd.py \
  --role top --serial TOP_SERIAL --output captures/top/frame_0001
python capture_realsense_rgbd.py \
  --role probe --serial PROBE_SERIAL --output captures/probe/frame_0001
```

Project an existing 2D centerline:

```bash
python project_rgbd.py \
  --centerline /path/to/centerline.json \
  --depth /path/to/depth.png \
  --intrinsics /path/to/intrinsics.json \
  --output /path/to/curve_3d
```

Segment and project an aligned RGB-D frame:

```bash
python infer_rgbd.py \
  --color /path/to/color.png \
  --depth /path/to/depth.png \
  --intrinsics /path/to/intrinsics.json \
  --output /path/to/output
```

Color and depth must have identical dimensions, and depth must already be aligned to color.

For the rail-mounted camera, pass the encoder position at image acquisition time. This emits `probe_alignment.json` in metres:

```bash
python infer_rgbd.py \
  --color /path/to/color.png \
  --depth /path/to/depth.png \
  --intrinsics /path/to/intrinsics.json \
  --config config/probe_d405_mount.yaml \
  --rail-position 0.0124 \
  --capture-stamp-ns 1786675200123456789 \
  --task-id inspection-001 \
  --sample-index 42 \
  --output /path/to/output
```

The rail position must come from the same acquisition timestamp, not from message arrival time. The probe center offset and minimum alignment point count are loaded from the camera profile unless explicitly overridden.

## Train and evaluate

Default training uses the active dataset and writes to the project-level results directory:

```bash
python train.py \
  --epochs 30 \
  --image-size 192 \
  --batch-size 8 \
  --base-channels 8 \
  --original-only
```

The original reproducible training command is also available as:

```bash
bash train_mvp_wsl.sh
```

Evaluate the delivered checkpoint:

```bash
python evaluate.py
python evaluate_centerline.py
```

Export the delivered checkpoint to the default ONNX model path:

```bash
python export_onnx.py
```

Every dataset, checkpoint, model, and output option remains overrideable on the command line.

## Tests

```bash
python -m unittest discover -s tests -v
```

## Current scope

- Input: RGB image/directory or one aligned RGB-D frame.
- Learned output: binary weld-area segmentation.
- Conventional outputs: weld edges and ordered 2D centerline points.
- RGB-D output: ordered 3D points in camera and robot surface frames.
- Supported deployment: Windows-hosted WSL and ordinary Linux.
- Not yet included here: continuous camera streaming, temporal tracking, calibration target solver, or direct ROS 2 publishing. ROS messages, rail safety control and timestamp interpolation live in the chassis workspace.
- Hardware limitation: real D405 depth accuracy and weld-arc robustness have not yet been validated.
