import os
import sys
import time

import cv2
import numpy as np


JETSON_INFERENCE_ROOT = "/workspace/jetson-inference"

WIDTH = 320
HEIGHT = 240
NETWORK = "fcn-mobilenet"
RUN_SECONDS = 20.0
WARMUP_FRAMES = 2
LOOP_PAUSE_SECONDS = 1.0

ENABLE_ARM_SIGNAL = True
DEPTH_THRESHOLD = 1.6
PAN_SERVO_ID = 1
TILT_SERVO_ID = 5
PAN_ANGLE = 10
TILT_ANGLE = 8
HOME_ANGLE = 0
SERVO_SPEED = 180
SERVO_SETTLE_SECONDS = 0.18


def setup_paths():
    paths = [
        f"{JETSON_INFERENCE_ROOT}/build/aarch64/lib/python/3.6",
        f"{JETSON_INFERENCE_ROOT}/python/examples",
    ]
    for path in paths:
        if path not in sys.path:
            sys.path.insert(0, path)

    lib_path = f"{JETSON_INFERENCE_ROOT}/build/aarch64/lib"
    os.environ["LD_LIBRARY_PATH"] = lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")


def setup_arm_signal():
    if not ENABLE_ARM_SIGNAL:
        print("[arm] disabled")
        return None

    try:
        from SCSCtrl import TTLServo
    except ImportError:
        print("[arm] SCSCtrl import failed; continuing without arm signal")
        return None

    print("[arm] enabled; using servoAngleCtrl() signal motions")
    return TTLServo


def servo_swing(ttl_servo, servo_id, angle, label):
    """Move one servo both ways, then return it to center."""
    if ttl_servo is None:
        print(f"[arm] {label} skipped")
        return

    ttl_servo.servoAngleCtrl(servo_id, angle, 1, SERVO_SPEED)
    time.sleep(SERVO_SETTLE_SECONDS)
    ttl_servo.servoAngleCtrl(servo_id, -angle, 1, SERVO_SPEED)
    time.sleep(SERVO_SETTLE_SECONDS)
    ttl_servo.servoAngleCtrl(servo_id, HOME_ANGLE, 1, SERVO_SPEED)
    time.sleep(SERVO_SETTLE_SECONDS)
    print(f"[arm] {label} swing complete")


def depth_signal_motion(ttl_servo, mean_depth):
    """Use the depth estimate to choose one of two visible signal motions."""
    if mean_depth < DEPTH_THRESHOLD:
        print(f"[decision] mean_depth={mean_depth:.3f} < {DEPTH_THRESHOLD}; pan left/right")
        servo_swing(ttl_servo, PAN_SERVO_ID, PAN_ANGLE, "pan")
    else:
        print(f"[decision] mean_depth={mean_depth:.3f} >= {DEPTH_THRESHOLD}; tilt up/down")
        servo_swing(ttl_servo, TILT_SERVO_ID, TILT_ANGLE, "tilt")


def summarize_center_depth(depth_array):
    h, w = depth_array.shape[:2]
    x1, x2 = int(w * 0.4), int(w * 0.6)
    y1, y2 = int(h * 0.4), int(h * 0.6)
    center = depth_array[y1:y2, x1:x2]
    return float(np.mean(center)), float(np.min(center)), float(np.max(center))


def main():
    setup_paths()

    from jetbot import Camera
    from jetson_inference import depthNet
    from jetson_utils import cudaDeviceSynchronize, cudaFromNumpy, cudaToNumpy

    ttl_servo = setup_arm_signal()

    print("[depth] loading network...")
    net = depthNet(NETWORK)
    depth_field = net.GetDepthField()
    depth_numpy = cudaToNumpy(depth_field)

    print(f"[depth] network={net.GetNetworkName()}")
    print(f"[depth] field={depth_field.width}x{depth_field.height}, format={depth_field.format}")

    print(f"[camera] starting JetBot Camera {WIDTH}x{HEIGHT}")
    camera = Camera.instance(width=WIDTH, height=HEIGHT)
    time.sleep(1.0)

    print(f"[warmup] processing {WARMUP_FRAMES} frame(s) before starting timer")
    warmup_done = 0
    while warmup_done < WARMUP_FRAMES:
        frame = camera.value
        if frame is None:
            print("[warmup] no camera frame")
            time.sleep(0.2)
            continue

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cuda_img = cudaFromNumpy(rgb)
        net.Process(cuda_img)
        cudaDeviceSynchronize()
        warmup_done += 1
        print(f"[warmup] frame {warmup_done}/{WARMUP_FRAMES} processed")
        time.sleep(0.1)

    successful_frames = 0
    start_time = time.time()
    previous_mean_depth = None

    try:
        while time.time() - start_time < RUN_SECONDS:
            frame = camera.value
            if frame is None:
                print("[camera] no frame")
                time.sleep(0.2)
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cuda_img = cudaFromNumpy(rgb)

            net.Process(cuda_img)
            cudaDeviceSynchronize()

            successful_frames += 1
            mean_depth, min_depth, max_depth = summarize_center_depth(depth_numpy)
            now = time.time()
            elapsed = now - start_time
            if previous_mean_depth is None:
                delta_text = "delta=n/a"
            else:
                delta_text = f"delta={mean_depth - previous_mean_depth:+.3f}"

            print(
                f"[{elapsed:05.2f}s frame={successful_frames:03d}] "
                f"center_depth mean={mean_depth:.3f}, min={min_depth:.3f}, "
                f"max={max_depth:.3f}, {delta_text}"
            )

            depth_signal_motion(ttl_servo, mean_depth)
            previous_mean_depth = mean_depth
            time.sleep(LOOP_PAUSE_SECONDS)

    finally:
        print(f"[demo] done; successful_frames={successful_frames}")
        try:
            camera.stop()
            time.sleep(1.0)
            print("[camera] stopped")
        except Exception as exc:
            print(f"[camera] stop failed: {exc}")


if __name__ == "__main__":
    main()
