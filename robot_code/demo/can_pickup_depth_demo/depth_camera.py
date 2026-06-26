import os
import sys
import time

import cv2
import numpy as np


JETSON_INFERENCE_ROOT = "/workspace/jetson-inference"


def setup_jetson_inference_paths(root=JETSON_INFERENCE_ROOT):
    """Add common jetson-inference build paths when running from /workspace."""
    paths = [
        os.path.join(root, "build/aarch64/lib/python/3.6"),
        os.path.join(root, "python/examples"),
    ]
    for path in paths:
        if path not in sys.path:
            sys.path.insert(0, path)

    lib_path = os.path.join(root, "build/aarch64/lib")
    os.environ["LD_LIBRARY_PATH"] = lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")


def summarize_region(depth_array, x1_ratio=0.4, y1_ratio=0.4, x2_ratio=0.6, y2_ratio=0.6):
    """Return mean/min/max for a normalized region in the depth field."""
    height, width = depth_array.shape[:2]
    x1 = max(0, min(width - 1, int(width * x1_ratio)))
    x2 = max(x1 + 1, min(width, int(width * x2_ratio)))
    y1 = max(0, min(height - 1, int(height * y1_ratio)))
    y2 = max(y1 + 1, min(height, int(height * y2_ratio)))

    region = depth_array[y1:y2, x1:x2]
    finite = region[np.isfinite(region)]
    if finite.size == 0:
        return {
            "mean": 0.0,
            "min": 0.0,
            "max": 0.0,
            "count": 0,
            "region": (x1, y1, x2, y2),
        }

    return {
        "mean": float(np.mean(finite)),
        "min": float(np.min(finite)),
        "max": float(np.max(finite)),
        "count": int(finite.size),
        "region": (x1, y1, x2, y2),
    }


class DepthCamera:
    """Small wrapper around JetBot Camera and jetson-inference depthNet."""

    def __init__(self, width=320, height=240, network="fcn-mobilenet"):
        setup_jetson_inference_paths()

        from jetbot import Camera
        from jetson_inference import depthNet
        from jetson_utils import cudaDeviceSynchronize, cudaFromNumpy, cudaToNumpy

        self.width = width
        self.height = height
        self.network = network
        self._camera_cls = Camera
        self._cuda_from_numpy = cudaFromNumpy
        self._cuda_sync = cudaDeviceSynchronize

        print("[depth] loading network {}".format(network))
        self.net = depthNet(network)
        self.depth_field = self.net.GetDepthField()
        self.depth_array = cudaToNumpy(self.depth_field)
        self.camera = None

    def start(self, warmup_frames=2):
        print("[camera] starting JetBot Camera {}x{}".format(self.width, self.height))
        try:
            self.camera = self._camera_cls.instance(width=self.width, height=self.height)
        except RuntimeError as exc:
            self.camera = None
            print("[camera] failed to initialize JetBot Camera")
            print("[camera] common fixes: stop other notebook kernels, restart this kernel, or restart nvargus-daemon")
            raise
        time.sleep(1.0)

        done = 0
        while done < warmup_frames:
            frame = self.read_frame()
            if frame is None:
                print("[warmup] no camera frame")
                time.sleep(0.2)
                continue
            self.process_frame(frame)
            done += 1
            print("[warmup] frame {}/{} processed".format(done, warmup_frames))
            time.sleep(0.1)

    def stop(self):
        if self.camera is None:
            return
        try:
            self.camera.stop()
            time.sleep(0.5)
            print("[camera] stopped")
        finally:
            self.camera = None

    def read_frame(self):
        if self.camera is None:
            return None
        return self.camera.value

    def process_frame(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cuda_img = self._cuda_from_numpy(rgb)
        self.net.Process(cuda_img)
        self._cuda_sync()
        return self.depth_array

    def observe(self, region=(0.4, 0.4, 0.6, 0.6)):
        frame = self.read_frame()
        if frame is None:
            return None
        depth = self.process_frame(frame)
        stats = summarize_region(depth, region[0], region[1], region[2], region[3])
        stats["timestamp"] = time.time()
        return stats

    def observe_many(self, seconds=3.0, interval=0.5, region=(0.4, 0.4, 0.6, 0.6)):
        start = time.time()
        results = []
        while time.time() - start < seconds:
            stats = self.observe(region)
            if stats is not None:
                results.append(stats)
            time.sleep(interval)
        return results


def mean_of_stats(stats_list, key="mean"):
    values = [item[key] for item in stats_list if item is not None and item.get("count", 0) > 0]
    if not values:
        return None
    return float(np.mean(values))
