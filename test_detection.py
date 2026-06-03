import argparse
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(PROJECT_ROOT / ".cache" / "matplotlib"),
)

import cv2

try:
    from ultralytics import YOLO  # type: ignore[import]
except ImportError as exc:
    raise ImportError(
        "The ultralytics package is required. Install it with `pip install ultralytics`."
    ) from exc

DEFAULT_MODEL_PATH = PROJECT_ROOT / "best.pt"


def parse_args():
    parser = argparse.ArgumentParser(description="Run YOLO detection from a camera feed.")
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL_PATH,
        help="Path to a YOLO .pt file or a directory containing one. Default: best.pt.",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera index to use. Default: 0.",
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=0.5,
        help="Minimum confidence required to display a detection. Default: 0.5.",
    )
    return parser.parse_args()


def resolve_model_path(model_path):
    if model_path.is_file():
        return model_path

    if model_path.is_dir():
        model_files = sorted(
            model_path.rglob("*.pt"),
            key=lambda path: (
                path.name != "best.pt",
                path.name != "last.pt",
                str(path),
            ),
        )
        if model_files:
            return model_files[0]

    raise RuntimeError(
        f"No YOLO .pt model found at {model_path}. "
        "Train a model first, then place best.pt in the project root or pass "
        "--model path/to/best.pt."
    )


def draw_detections(frame, results, confidence_threshold):
    detected_boxes = []

    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()

        for box, cls, confidence in zip(boxes, classes, confidences):
            if confidence < confidence_threshold:
                continue

            x1, y1, x2, y2 = map(int, box[:4])
            label = f"{result.names[int(cls)]} {confidence:.2f}"
            detected_boxes.append((x1, y1, x2, y2))
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                label,
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

    return detected_boxes


def main():
    args = parse_args()
    model_path = resolve_model_path(args.model)
    yolo_model = YOLO(str(model_path))
    camera = cv2.VideoCapture(args.camera)

    if not camera.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Could not read a frame from the camera.")

            results = yolo_model(frame, conf=args.confidence, verbose=False)
            draw_detections(frame, results, args.confidence)

            cv2.imshow("Paper Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
