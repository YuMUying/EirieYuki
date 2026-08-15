# WES Combined Dataset

This directory is a consolidated dataset for binary weld-area segmentation and weld-edge extraction.

## Included supervised sources

- `Weld-bead-images-and-masks`: 1,394 image/mask pairs.
- `welding_spots_100`: 100 image/mask pairs.

The `3D_Laser_Vision_Welding_Robot-main` directory is excluded. Detection-only datasets are not converted from rectangles into masks. Their exact-deduplicated images are stored under `auxiliary_unlabeled` for self-supervised learning, pseudo-labeling, or manual annotation.

## Layout

- `supervised/train`: original training pairs plus one offline augmentation per original.
- `supervised/val`: original validation pairs only.
- `supervised/test`: original test pairs only.
- `auxiliary_unlabeled/images`: exact-deduplicated images from the two detection datasets.
- `manifests`: source, split, group, transform, duplicate, and hash provenance.
- `splits`: sample IDs for each supervised split.

## Final counts

| Split | Original | Augmented | Total pairs |
| --- | ---: | ---: | ---: |
| Train | 1,046 | 1,046 | 2,092 |
| Validation | 225 | 0 | 225 |
| Test | 223 | 0 | 223 |

The auxiliary pool contains 2,056 exact-unique images, reduced from 3,507 detection-dataset files.

## Processing

- All supervised images are RGB PNG at 448 x 448.
- All supervised masks are single-channel binary PNG with values 0 and 255.
- Aspect ratio is preserved; non-square images are centered and padded.
- Split ratio targets are 70/15/15, stratified by source.
- Consecutive weld-bead frames are grouped by capture prefix before splitting.
- Welding-spots samples are grouped in sequential blocks of five before splitting.
- Seed: `20260813`.

## Training augmentation

Each original training pair has one deterministic augmented pair. Augmentation uses mild horizontal/vertical flips, rotation within 8 degrees, brightness/contrast adjustment, and low-amplitude Gaussian noise. Geometry is applied identically to image and mask. Validation and test samples are never augmented.

## Important

Only `supervised` contains WES ground truth. `auxiliary_unlabeled` must not be used as segmentation ground truth without pixel-level relabeling. Original source directories remain unchanged. Review `SOURCE_LICENSES.md` before redistribution or commercial use.
