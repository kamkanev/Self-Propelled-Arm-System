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
    parser.add_argument("--model", type=Path, default=Path("yolo11n.pt"), help="Base YOLO model path. yolo11n (nano, ~2.6M params) is the variant that runs in real time on the JETANK's Jetson Nano (4 GB shared CPU/GPU memory). Training still happens on the RTX 3060; only the model size is capped for the deployment target. Use yolo11s if the Nano has FPS headroom.")
    parser.add_argument("--data", type=Path, default=Path("paper_detect/data.yaml"), help="Dataset YAML path.")
    parser.add_argument("--epochs", type=int, default=100, help="Training epochs. Small dataset (226 imgs) benefits from many epochs; ")
    parser.add_argument("--patience", type=int, default=50, )
    parser.add_argument("--imgsz", type=int, default=416, help="Image size. Inference cost scales ~quadratically with imgsz, so this is the biggest FPS lever on the Jetson Nano. Train at the size you deploy at: 416 balances reach (spotting the paper far away) against real-time control. Drop to 320 for more FPS; raise to 640 only if you accept lower FPS or run TensorRT.")
    parser.add_argument("--batch", default=-1,)
    parser.add_argument("--workers", type=int, default=8, help="Data loader workers (parallel CPU data loading/augmentation).")
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


def parse_batch(value):
    """Batch may be an int (fixed) or -1 (Ultralytics auto-fit ~60% VRAM)."""
    try:
        return int(value)
    except (TypeError, ValueError):
        return value


def project_path(path):
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def main():
    args = parse_args()
    batch = parse_batch(args.batch)
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
        f"model={args.model} imgsz={args.imgsz} epochs={args.epochs} "
        f"batch={batch} workers={args.workers} cache={args.cache}"
    )
    model_path = project_path(args.model)
    data_path = project_path(args.data)
    project_path_value = project_path(args.project)

    model = YOLO(str(model_path))
    model.train(
        data=str(data_path),
        epochs=args.epochs,
        patience=args.patience,
        imgsz=args.imgsz,
        batch=batch,
        workers=args.workers,
        device=args.device,
        cache=(False if args.cache == "none" else args.cache),
        amp=True,           
        cos_lr=True,         # cosine LR decay -> better convergence on small datasets
        optimizer="auto",    # Ultralytics picks AdamW/SGD + LR for the dataset size
        close_mosaic=10,     # disable mosaic for final 10 epochs to sharpen boxes
        mixup=0.1,
        copy_paste=0.1,
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
