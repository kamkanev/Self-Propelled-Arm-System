import argparse
import os
import shutil
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(PROJECT_ROOT / ".cache" / "matplotlib"),
)

from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description="Train a low-memory paper detector.")
    parser.add_argument("--model", type=Path, default=Path("yolo11s.pt"), help="Base YOLO model path.")
    parser.add_argument("--data", type=Path, default=Path("paper_detect/data.yaml"), help="Dataset YAML path.")
    parser.add_argument("--epochs", type=int, default=50, help="Training epochs.")
    parser.add_argument("--imgsz", type=int, default=416, help="Image size.")
    parser.add_argument("--batch", type=int, default=1, help="Batch size.")
    parser.add_argument("--workers", type=int, default=0, help="Data loader workers.")
    parser.add_argument("--device", default="cpu", help="Training device.")
    parser.add_argument("--project", type=Path, default=Path("paper_detect/runs"), help="Output directory.")
    parser.add_argument("--name", default="train", help="Run name.")
    return parser.parse_args()


def project_path(path):
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def main():
    args = parse_args()
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
        cache=False,
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
