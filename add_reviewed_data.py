import argparse
import random
import shutil
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Merge reviewed images + YOLO labels from new_data/ into the training dataset."
    )
    parser.add_argument("--images", type=Path, default=PROJECT_ROOT / "new_data" / "images", help="Reviewed images dir.")
    parser.add_argument("--labels", type=Path, default=PROJECT_ROOT / "new_data" / "labels", help="Reviewed labels dir.")
    parser.add_argument("--dataset", type=Path, default=PROJECT_ROOT / "paper_detect", help="Dataset root (has train/ valid/).")
    parser.add_argument("--val-frac", type=float, required=True, help="REQUIRED. Fraction (0.0-1.0) of new images to send to valid/ instead of train/. e.g. 0.2 = 20%% to valid, 80%% to train; 0.0 = all to train.")
    parser.add_argument("--seed", type=int, default=0, help="Shuffle seed for the val split.")
    parser.add_argument("--dry-run", action="store_true", help="Show what would be copied without copying.")
    return parser.parse_args()


def main():
    args = parse_args()
    if not 0.0 <= args.val_frac <= 1.0:
        raise SystemExit(f"--val-frac must be between 0.0 and 1.0 (got {args.val_frac}).")
    if not args.images.is_dir():
        raise SystemExit(f"Images dir not found: {args.images}")

    images = sorted(p for p in args.images.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    if not images:
        raise SystemExit(f"No images in {args.images}")

    random.seed(args.seed)
    val_count = round(len(images) * args.val_frac)
    val_set = set(random.sample(images, val_count)) if val_count else set()

    counts = {"train": 0, "valid": 0, "skipped": 0}
    for img in images:
        label = args.labels / f"{img.stem}.txt"
        if not label.exists():
            print(f"SKIP (no label): {img.name}")
            counts["skipped"] += 1
            continue
        split = "valid" if img in val_set else "train"
        dst_img = args.dataset / split / "images" / img.name
        dst_lbl = args.dataset / split / "labels" / label.name
        if args.dry_run:
            print(f"{split}: {img.name} (+ {label.name})")
        else:
            dst_img.parent.mkdir(parents=True, exist_ok=True)
            dst_lbl.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(img, dst_img)
            shutil.copy2(label, dst_lbl)
        counts[split] += 1

    print(f"\nDone. train += {counts['train']}, valid += {counts['valid']}, skipped = {counts['skipped']}")
    if not args.dry_run and (counts["train"] or counts["valid"]):
        print("Now retrain on the expanded dataset:  python train_paper.py")


if __name__ == "__main__":
    main()
