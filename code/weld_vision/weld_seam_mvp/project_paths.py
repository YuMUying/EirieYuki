from pathlib import Path


CODE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CODE_DIR.parents[2]

WELD_DATASET = PROJECT_ROOT / "datasets" / "weld_vision" / "WES-Combined-Dataset"
WELD_MODEL_DIR = PROJECT_ROOT / "models" / "weld_vision" / "segmentation"
WELD_RESULTS_DIR = PROJECT_ROOT / "results" / "weld_vision"

DEFAULT_ONNX_MODEL = WELD_MODEL_DIR / "weld_segmentation.onnx"
DEFAULT_CHECKPOINT = WELD_MODEL_DIR / "weld_segmentation.pt"
DEFAULT_CAMERA_CONFIG = CODE_DIR / "config" / "d405_mount.yaml"
