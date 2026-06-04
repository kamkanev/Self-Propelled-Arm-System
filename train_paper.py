import argparse
import os
import shutil
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(PROJECT_ROOT / ".cache" / "matplotlib"),
)

import torch

try:
    import psutil

    _PHYSICAL_CORES = psutil.cpu_count(logical=False) or os.cpu_count() or 1
except ImportError:
    _PHYSICAL_CORES = os.cpu_count() or 1

from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description="Train a low-memory paper detector.")
    parser.add_argument("--model", type=Path, default=Path("yolo11s.pt"), help="Base YOLO model path.")
    parser.add_argument("--data", type=Path, default=Path("paper_detect/data.yaml"), help="Dataset YAML path.")
    parser.add_argument("--epochs", type=int, default=50, help="Training epochs.")
    parser.add_argument("--imgsz", type=int, default=416, help="Image size.")
    parser.add_argument("--batch", type=int, default=16, help="Batch size. On the RTX 3060 Laptop (6 GB VRAM) at imgsz 416, 16 is a safe fit; try 24-32 if VRAM allows, or -1 to let Ultralytics auto-pick (~60%% VRAM).")
    parser.add_argument("--workers", type=int, default=2, help="Data loader workers (parallel CPU data loading/augmentation).")
    parser.add_argument("--device", default="0", help="Training device. '0' = first CUDA GPU (RTX 3060 Laptop). Pass 'cpu' to force CPU.")
    parser.add_argument(
        "--threads",
        type=int,
        default=_PHYSICAL_CORES,
        help="PyTorch CPU compute threads (intra-op). Default: physical core count.",
    )
    parser.add_argument(
        "--cache",
        choices=["ram", "disk", "none"],
        default="ram",
        help="Cache images to speed up every epoch after the first. Default: ram (32 GB on the training machine easily holds this dataset).",
    )
    parser.add_argument("--project", type=Path, default=Path("paper_detect/runs"), help="Output directory.")
    parser.add_argument("--name", default="train", help="Run name.")
    return parser.parse_args()


def project_path(path):
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def main():
    args = parse_args()
    torch.set_num_threads(args.threads)
    cuda_ok = torch.cuda.is_available()
    if str(args.device) != "cpu" and not cuda_ok:
        print(
            f"[train] WARNING: device={args.device} requested but CUDA is unavailable. "
            "Install the CUDA build of torch (pip install -r requirements-gpu.txt) "
            "or pass --device cpu."
        )
    gpu = torch.cuda.get_device_name(0) if cuda_ok else "none"
    print(
        f"[train] device={args.device} gpu={gpu} threads={args.threads} "
        f"batch={args.batch} workers={args.workers} cache={args.cache}"
    )
    model_path = project_path(args.model)
    data_path = project_path(args.data)
    project_path_value = project_path(args.project)

    model = YOLO(str(model_path))
    model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device=args.device,
        cache=(False if args.cache == "none" else args.cache),
        project=str(project_path_value),
        name=args.name,
        exist_ok=True,
    )

    trained_best = project_path_value / args.name / "weights" / "best.pt"
    default_best = PROJECT_ROOT / "best.pt"
    if trained_best.exists():
        shutil.copy2(trained_best, default_best)
        print(f"Copied trained model to {default_best}")


if __name__ == "__main__":
    main()
