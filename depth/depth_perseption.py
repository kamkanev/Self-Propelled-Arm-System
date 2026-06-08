import argparse
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(PROJECT_ROOT / ".cache" / "matplotlib"),
)
os.environ.setdefault("TORCH_HOME", str(PROJECT_ROOT / ".cache" / "torch"))

import cv2
import numpy as np
import torch


def parse_args():
    parser = argparse.ArgumentParser(description="Quick MiDaS relative-depth camera test.")
    parser.add_argument("--camera", type=int, default=0, help="Camera index. Default: 0.")
    parser.add_argument(
        "--model",
        default="MiDaS_small",
        choices=("MiDaS_small", "DPT_Hybrid", "DPT_Large"),
        help="MiDaS model type. Default: MiDaS_small.",
    )
    parser.add_argument(
        "--device",
        default="auto",
        choices=("auto", "cpu", "cuda"),
        help="Device to run on. Default: auto.",
    )
    return parser.parse_args()


def get_device(device_name):
    if device_name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device_name == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested, but torch.cuda.is_available() is false.")
    return torch.device(device_name)


def load_midas(model_type, device):
    model = torch.hub.load("intel-isl/MiDaS", model_type)
    model.to(device)
    model.eval()

    transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
    if model_type == "MiDaS_small":
        transform = transforms.small_transform
    else:
        transform = transforms.dpt_transform

    return model, transform


def estimate_depth(frame, model, transform, device):
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    input_batch = transform(rgb_frame).to(device)

    with torch.no_grad():
        prediction = model(input_batch)
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=rgb_frame.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()

    return prediction.cpu().numpy()


def colorize_depth(depth_map):
    normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
    depth_uint8 = normalized.astype(np.uint8)
    return cv2.applyColorMap(depth_uint8, cv2.COLORMAP_MAGMA)


def run_camera(camera_index, model_type, device_name):
    device = get_device(device_name)
    model, transform = load_midas(model_type, device)
    camera = cv2.VideoCapture(camera_index)

    if not camera.isOpened():
        raise RuntimeError(f"Could not open camera index {camera_index}.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Could not read a frame from the camera.")

            depth_map = estimate_depth(frame, model, transform, device)
            depth_preview = colorize_depth(depth_map)

            cv2.imshow("Camera", frame)
            cv2.imshow("MiDaS Relative Depth", depth_preview)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


def main():
    args = parse_args()
    run_camera(args.camera, args.model, args.device)


if __name__ == "__main__":
    main()
