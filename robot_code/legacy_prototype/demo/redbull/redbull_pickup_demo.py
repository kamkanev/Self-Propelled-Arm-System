"""Minimal Red Bull pickup flow demo.

This is a meeting/demo script, not a calibrated robot program.  It intentionally
uses few abstractions so the whole planned flow is easy to read in one file.

What is real in this file:
- the intended state order for search, align, approach, stop, and arm pickup
- the local model files staged under ./models for future real inference
- the servo target logging format for future calibration

What is disabled:
- tracked-base movement
- physical arm movement when DRY_RUN is True
"""

import time
from pathlib import Path


# False means the arm will really call TTLServo.servoAngleCtrl().
DRY_RUN = False

# Keep the tracked base still. Manually place the robot near the can/bottle and
# only test the perception-to-arm part of the demo.
MANUAL_PLACEMENT_MODE = True

# No command-line input is needed. Edit these constants for quick tests.
MONITOR_SECONDS = 30.0
MONITOR_INTERVAL_SECONDS = 2.0

LOG_PATH = Path(__file__).with_name("redbull_servo_positions_log.csv")
DEMO_DIR = Path(__file__).resolve().parent
MODELS_DIR = DEMO_DIR / "models"
LOG_COLUMNS = [
    "timestamp",
    "event_type",
    "step",
    "source",
    "servo_id",
    "target",
    "speed",
    "found",
    "confidence",
    "distance_m",
    "horizontal_offset_m",
    "bbox_center_x",
    "bbox_height_ratio",
    "model_path",
    "model_exists",
    "note",
]

# Same DepthNet family as demo/depthnet_servo_decision_demo.ipynb:
# NETWORK = "fcn-mobilenet".
DEPTH_NETWORK_NAME = "fcn-mobilenet"
DEPTH_MODEL_PATH = MODELS_DIR / "depthnet" / "monodepth_fcn_mobilenet.onnx"
DEPTH_ENGINE_PATH = MODELS_DIR / "depthnet" / "monodepth_fcn_mobilenet.onnx.1.1.7103.GPU.FP16.engine"

# New object-recognition weight supplied for this Red Bull demo.
BOTTLE_RECOGNITION_MODEL_PATH = MODELS_DIR / "bottle_recognition" / "ssd-mobilenet.onnx"
BOTTLE_RECOGNITION_LABELS_PATH = MODELS_DIR / "bottle_recognition" / "labels.txt"
BOTTLE_RECOGNITION_REFERENCE_DIR = MODELS_DIR / "bottle_recognition" / "reference"
BOTTLE_RECOGNITION_INPUT_BLOB = "input_0"
BOTTLE_RECOGNITION_OUTPUT_CVG = "scores"
BOTTLE_RECOGNITION_OUTPUT_BBOX = "boxes"
BOTTLE_RECOGNITION_THRESHOLD = 0.30
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# Approximate European small Red Bull can: 250 ml slim aluminium can.
CAN_VOLUME_ML = 250.0
CAN_DIAMETER_MM = 53.0
CAN_HEIGHT_MM = 133.0
CAN_EMPTY_MASS_G = 11.0
CAN_FULL_MASS_G_ESTIMATE = (261.0, 271.0)

# Coarse navigation thresholds. These are placeholders for real calibration.
CENTER_OFFSET_TOLERANCE_M = 0.08
STAGING_DISTANCE_M = 0.75


class CanObservation:
    """Predicted can data that will later come from detectNet + DepthNet."""

    def __init__(
        self,
        found,
        distance_m,
        horizontal_offset_m,
        bbox_center_x,
        bbox_height_ratio,
        confidence,
    ):
        self.found = found
        self.distance_m = distance_m
        self.horizontal_offset_m = horizontal_offset_m
        self.bbox_center_x = bbox_center_x
        self.bbox_height_ratio = bbox_height_ratio
        self.confidence = confidence


last_bottle_confidence = None

ttl_servo = None


def csv_value(value):
    """Format a value for the simple CSV log."""
    if value is None:
        return ""
    text = str(value)
    if any(ch in text for ch in [",", '"', "\n"]):
        text = '"' + text.replace('"', '""') + '"'
    return text


def append_log(row):
    """Append one event row without adding extra package dependencies."""
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(",".join(csv_value(row.get(column, "")) for column in LOG_COLUMNS) + "\n")


def describe_staged_models():
    """Print the model files copied beside this demo for future integration."""
    print("[models] staged files for future real inference:")
    print(f"[models] depth network name used by prior notebook: {DEPTH_NETWORK_NAME!r}")
    print(
        "[models] bottle recognition blobs inferred from jetson-inference export script: "
        f"input={BOTTLE_RECOGNITION_INPUT_BLOB}, "
        f"confidence={BOTTLE_RECOGNITION_OUTPUT_CVG}, "
        f"bbox={BOTTLE_RECOGNITION_OUTPUT_BBOX}"
    )
    for label, path, source in [
        ("depth model copied from prior working DepthNet setup", DEPTH_MODEL_PATH, "depthnet"),
        ("depth engine cache copied from prior working DepthNet setup", DEPTH_ENGINE_PATH, "depthnet"),
        ("bottle recognition model supplied for this demo", BOTTLE_RECOGNITION_MODEL_PATH, "bottle_recognition"),
        ("bottle recognition labels expected by exporter", BOTTLE_RECOGNITION_LABELS_PATH, "bottle_recognition"),
    ]:
        exists = path.exists()
        if exists:
            size_mb = path.stat().st_size / (1024 * 1024)
            print(f"[models] OK      {label}: {path} ({size_mb:.2f} MB)")
        else:
            print(f"[models] MISSING {label}: {path}")
            size_mb = ""

        append_log(
            {
                "timestamp": f"{time.time():.3f}",
                "event_type": "model_file",
                "step": "startup",
                "source": source,
                "model_path": path,
                "model_exists": exists,
                "note": f"{label}; size_mb={size_mb}",
            }
        )

    if BOTTLE_RECOGNITION_REFERENCE_DIR.exists():
        print(f"[models] reference files copied from jetson-inference: {BOTTLE_RECOGNITION_REFERENCE_DIR}")


def init_servo_if_needed():
    """Load JETANK servo API only when DRY_RUN is disabled."""
    global ttl_servo
    if DRY_RUN:
        return
    from SCSCtrl import TTLServo

    ttl_servo = TTLServo


def ensure_servo_log():
    """Create the external log used for model checks, predictions, and servos."""
    expected_header = ",".join(LOG_COLUMNS)
    if LOG_PATH.exists():
        with LOG_PATH.open("r", encoding="utf-8") as handle:
            current_header = handle.readline().strip()
        if current_header == expected_header:
            return
        legacy_path = LOG_PATH.with_name(
            LOG_PATH.stem + "_legacy_" + time.strftime("%Y%m%d_%H%M%S") + LOG_PATH.suffix
        )
        LOG_PATH.replace(legacy_path)
        print(f"[log] existing old-format log moved to: {legacy_path}")

    with LOG_PATH.open("w", encoding="utf-8") as handle:
        handle.write(expected_header + "\n")

    if LOG_PATH.exists():
        return


def log_servo_target(step, servo_id, target, speed, note=""):
    """Record every intended servo target, even in dry-run mode."""
    append_log(
        {
            "timestamp": f"{time.time():.3f}",
            "event_type": "servo",
            "step": step,
            "source": "arm",
            "servo_id": servo_id,
            "target": target,
            "speed": speed,
            "note": note,
        }
    )


def log_status(step, source, note):
    append_log(
        {
            "timestamp": f"{time.time():.3f}",
            "event_type": "status",
            "step": step,
            "source": source,
            "note": note,
        }
    )


def move_servo(step, servo_id, target, speed, note=""):
    """Send or print one servoAngleCtrl target."""
    log_servo_target(step, servo_id, target, speed, note)
    if ttl_servo is not None:
        # JETANK tutorial style: servoAngleCtrl(servoID, angleInput, direction, speed).
        ttl_servo.servoAngleCtrl(servo_id, target, 1, speed)
    time.sleep(0.25)


def apply_pose(step, targets, speed, note=""):
    """Apply a tiny placeholder pose and log all servo targets."""
    for servo_id, target in targets:
        move_servo(step, servo_id, target, speed, note)
    time.sleep(0.4)


def base_set_velocity(linear, angular, seconds):
    """Tracked base movement is disabled for hand-placed pickup tests."""
    print(
        "[base] movement disabled; "
        f"requested linear={linear:+.2f} angular={angular:+.2f} for {seconds:.2f}s"
    )


def base_stop():
    print("[base] stop")


def print_model_prediction(label, observation, source, note):
    """Print and log one real perception observation."""
    global last_bottle_confidence
    if last_bottle_confidence is None:
        confidence_delta = ""
    else:
        confidence_delta = observation.confidence - last_bottle_confidence
    last_bottle_confidence = observation.confidence

    append_log(
        {
            "timestamp": f"{time.time():.3f}",
            "event_type": "prediction",
            "step": label,
            "source": source,
            "found": observation.found,
            "confidence": f"{observation.confidence:.3f}",
            "distance_m": f"{observation.distance_m:.3f}",
            "horizontal_offset_m": f"{observation.horizontal_offset_m:.3f}",
            "bbox_center_x": f"{observation.bbox_center_x:.3f}",
            "bbox_height_ratio": f"{observation.bbox_height_ratio:.3f}",
            "model_path": BOTTLE_RECOGNITION_MODEL_PATH,
            "model_exists": BOTTLE_RECOGNITION_MODEL_PATH.exists(),
            "note": f"{note}; bottle_conf_delta={confidence_delta}",
        }
    )

    if not observation.found:
        print(
            f"[vision:{label}] bottle_found=False | bottle_conf={observation.confidence:.3f} "
            "| depth_mean_m=n/a"
        )
        return

    if confidence_delta == "":
        delta_text = "n/a"
    else:
        delta_text = f"{confidence_delta:+.3f}"
    print(
        f"[vision:{label}] DEPTH depth_mean_m={observation.distance_m:.3f} "
        f"| horizontal_offset_norm={observation.horizontal_offset_m:+.3f} "
        f"| bbox_center_x={observation.bbox_center_x:.3f} "
        f"| bbox_height_ratio={observation.bbox_height_ratio:.3f} "
        f"| BOTTLE found=True confidence={observation.confidence:.3f} "
        f"delta={delta_text}"
    )


def read_labels():
    if not BOTTLE_RECOGNITION_LABELS_PATH.exists():
        return []
    with BOTTLE_RECOGNITION_LABELS_PATH.open("r", encoding="utf-8") as handle:
        return [line.strip() for line in handle.readlines() if line.strip()]


def detection_label(class_id, labels):
    try:
        return labels[int(class_id)]
    except Exception:
        return ""


def is_target_detection(detection, labels):
    if not labels:
        return True
    label = detection_label(getattr(detection, "ClassID", -1), labels).lower()
    if not label:
        return True
    return "bottle" in label or "can" in label or "drink" in label


def detection_bbox(detection):
    left = float(getattr(detection, "Left", 0.0))
    top = float(getattr(detection, "Top", 0.0))
    right = float(getattr(detection, "Right", 0.0))
    bottom = float(getattr(detection, "Bottom", 0.0))
    return left, top, right, bottom


def summarize_depth_region(depth_array, left_norm, top_norm, right_norm, bottom_norm):
    import numpy as np

    height, width = depth_array.shape[:2]
    left = max(0, min(width - 1, int(left_norm * width)))
    right = max(left + 1, min(width, int(right_norm * width)))
    top = max(0, min(height - 1, int(top_norm * height)))
    bottom = max(top + 1, min(height, int(bottom_norm * height)))
    region = depth_array[top:bottom, left:right]
    finite = region[np.isfinite(region)]
    if finite.size == 0:
        return 0.0, 0.0, 0.0
    return float(finite.mean()), float(finite.min()), float(finite.max())


def make_observation_from_frame(frame, depth_net, depth_numpy, detector, labels, cuda_from_numpy, cuda_sync):
    import cv2

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    depth_img = cuda_from_numpy(rgb)
    depth_net.Process(depth_img)
    cuda_sync()

    detections = []
    if detector is not None:
        rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        detect_img = cuda_from_numpy(rgba)
        detections = detector.Detect(detect_img)

    frame_h, frame_w = frame.shape[:2]
    target_detections = [det for det in detections if is_target_detection(det, labels)]
    best = None
    if target_detections:
        best = max(target_detections, key=lambda det: float(getattr(det, "Confidence", 0.0)))

    if best is None:
        mean_depth, min_depth, max_depth = summarize_depth_region(depth_numpy, 0.40, 0.40, 0.60, 0.60)
        observation = CanObservation(False, mean_depth, 0.0, 0.5, 0.0, 0.0)
        note = (
            f"real depth center region; detections={len(detections)}; "
            f"target_detections=0; depth_min={min_depth:.3f}; depth_max={max_depth:.3f}"
        )
        return observation, note

    left, top, right, bottom = detection_bbox(best)
    left_norm = max(0.0, min(1.0, left / frame_w))
    right_norm = max(0.0, min(1.0, right / frame_w))
    top_norm = max(0.0, min(1.0, top / frame_h))
    bottom_norm = max(0.0, min(1.0, bottom / frame_h))
    center_x = (left_norm + right_norm) / 2.0
    bbox_height_ratio = max(0.0, bottom_norm - top_norm)
    horizontal_offset_norm = center_x - 0.5
    mean_depth, min_depth, max_depth = summarize_depth_region(
        depth_numpy,
        max(0.0, center_x - 0.08),
        max(0.0, ((top_norm + bottom_norm) / 2.0) - 0.08),
        min(1.0, center_x + 0.08),
        min(1.0, ((top_norm + bottom_norm) / 2.0) + 0.08),
    )
    class_id = int(getattr(best, "ClassID", -1))
    label = detection_label(class_id, labels)
    confidence = float(getattr(best, "Confidence", 0.0))
    observation = CanObservation(True, mean_depth, horizontal_offset_norm, center_x, bbox_height_ratio, confidence)
    note = (
        f"real detector/depth; detections={len(detections)}; class_id={class_id}; "
        f"label={label}; bbox=({left:.1f},{top:.1f},{right:.1f},{bottom:.1f}); "
        f"depth_min={min_depth:.3f}; depth_max={max_depth:.3f}"
    )
    return observation, note


def load_real_vision_modules():
    print("[vision] loading camera, DepthNet, and bottle detector")
    from jetbot import Camera
    from jetson_inference import depthNet, detectNet
    from jetson_utils import cudaDeviceSynchronize, cudaFromNumpy, cudaToNumpy

    labels = read_labels()
    if labels:
        print(f"[vision] labels loaded: {len(labels)} from {BOTTLE_RECOGNITION_LABELS_PATH}")
    else:
        print("[vision] labels missing or empty; raw detections will not be class-filtered")

    depth_net = depthNet(DEPTH_NETWORK_NAME)
    depth_field = depth_net.GetDepthField()
    depth_numpy = cudaToNumpy(depth_field)

    detector = None
    if BOTTLE_RECOGNITION_MODEL_PATH.exists():
        detector_kwargs = {
            "model": str(BOTTLE_RECOGNITION_MODEL_PATH),
            "input_blob": BOTTLE_RECOGNITION_INPUT_BLOB,
            "output_cvg": BOTTLE_RECOGNITION_OUTPUT_CVG,
            "output_bbox": BOTTLE_RECOGNITION_OUTPUT_BBOX,
            "threshold": BOTTLE_RECOGNITION_THRESHOLD,
        }
        if BOTTLE_RECOGNITION_LABELS_PATH.exists():
            detector_kwargs["labels"] = str(BOTTLE_RECOGNITION_LABELS_PATH)
        print("[vision] loading detector ONNX")
        detector = detectNet(**detector_kwargs)
    else:
        print("[vision] bottle detector model missing; DepthNet-only monitoring")

    print(f"[camera] starting JetBot Camera {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
    camera = Camera.instance(width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
    time.sleep(1.0)
    return camera, depth_net, depth_numpy, detector, labels, cudaFromNumpy, cudaDeviceSynchronize


def monitor_manual_target(seconds):
    """Run real camera, DepthNet, and bottle-recognition monitoring."""
    print(f"[monitor] start {seconds:.0f}s manual vision monitor; no tracked-base movement.")
    log_status("monitor_start", "vision", f"seconds={seconds}; interval={MONITOR_INTERVAL_SECONDS}")
    camera = None
    start_time = time.time()
    frame_index = 0
    observation = None
    camera, depth_net, depth_numpy, detector, labels, cuda_from_numpy, cuda_sync = load_real_vision_modules()

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > seconds and frame_index > 0:
                break

            frame = camera.value
            if frame is None:
                print("[vision:monitor] no camera frame")
                log_status("monitor_no_frame", "vision", "camera.value is None")
            else:
                observation, note = make_observation_from_frame(
                    frame,
                    depth_net,
                    depth_numpy,
                    detector,
                    labels,
                    cuda_from_numpy,
                    cuda_sync,
                )
                print_model_prediction("monitor", observation, "real_bottle_recognition_plus_depthnet", note)
                frame_index += 1

            remaining = seconds - (time.time() - start_time)
            if remaining <= 0:
                break
            time.sleep(min(MONITOR_INTERVAL_SECONDS, remaining))
    finally:
        if camera is not None:
            camera.stop()

    log_status("monitor_end", "vision", f"frames={frame_index}")
    print(f"[monitor] done; frames={frame_index}")
    return observation


def reset_arm(step):
    """Return all servos to the simple tutorial neutral pose."""
    print(f"[arm] {step}: reset servos 1-5 to neutral")
    log_status(step, "arm", "reset servos 1-5 to 0 using servoAngleCtrl")
    apply_pose(step, [(1, 0), (2, 0), (3, 0), (4, 0), (5, 0)], speed=150, note="tutorial neutral reset")


def pickup_can(observation):
    """Placeholder arm pickup using predicted distance/offset from perception."""
    print("[state] PICK")
    base_stop()
    print(
        "[arm] received final model-derived target: "
        f"distance={observation.distance_m:.2f}m, "
        f"lateral_offset={observation.horizontal_offset_m:+.2f}m, "
        f"can_height={CAN_HEIGHT_MM:.0f}mm, can_diameter={CAN_DIAMETER_MM:.0f}mm"
    )

    # Placeholder poses. Keep these small until real safe poses are calibrated.
    print("[arm] pickup sequence start")
    apply_pose("home", [(1, 0), (5, 0)], speed=120, note="camera/arm neutral")
    apply_pose("pre_grasp", [(2, 18), (3, -12)], speed=100, note="low-risk approach pose")
    move_servo("open_gripper", 4, 0, speed=120, note="open before approach")
    apply_pose("reach", [(2, 26), (3, -22)], speed=80, note="move toward can")

    # Bias toward a firm grasp for a 250 ml can. This is not force-controlled.
    move_servo("firm_close_1", 4, -45, speed=150, note="first firm close")
    time.sleep(0.7)
    move_servo("firm_close_2", 4, -55, speed=120, note="settle and re-close")

    apply_pose("lift", [(2, 5), (3, 18)], speed=80, note="lift can for inspection")
    print("[arm] pickup attempt complete; stop here for manual inspection")


def main():
    ensure_servo_log()
    init_servo_if_needed()

    print("[demo] start 250 ml Red Bull pickup prototype")
    print(f"[demo] dry_run={DRY_RUN}")
    print(f"[demo] manual_placement_mode={MANUAL_PLACEMENT_MODE}")
    print(
        "[demo] can model: "
        f"{CAN_VOLUME_ML:.0f}ml, diameter={CAN_DIAMETER_MM:.0f}mm, "
        f"height={CAN_HEIGHT_MM:.0f}mm, empty={CAN_EMPTY_MASS_G:.1f}g, "
        f"full_estimate={CAN_FULL_MASS_G_ESTIMATE[0]:.0f}-{CAN_FULL_MASS_G_ESTIMATE[1]:.0f}g"
    )
    describe_staged_models()
    print("[demo] NOTE: monitor predictions are read from real camera/depth/detector modules.")
    print("[demo] NOTE: DepthNet route follows the previous working fcn-mobilenet notebook.")
    print("[demo] NOTE: do not trust class labels until the vision team confirms class order.")

    reset_arm("start_reset")

    try:
        if MANUAL_PLACEMENT_MODE:
            print("[demo] tracked base movement disabled; manually place robot near target.")
            observation = monitor_manual_target(MONITOR_SECONDS)
            if observation is None:
                raise RuntimeError("no valid real vision observation collected during monitor window")
        else:
            raise RuntimeError("automatic tracked-base mode is disabled in this real-module test script")
        pickup_can(observation)
    finally:
        reset_arm("end_reset")

    print(f"[demo] finished; servo log: {LOG_PATH}")


if __name__ == "__main__":
    main()
