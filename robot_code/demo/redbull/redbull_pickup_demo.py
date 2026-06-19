"""Minimal Red Bull pickup flow demo.

This is a meeting/demo script, not a calibrated robot program.  It intentionally
uses few abstractions so the whole planned flow is easy to read in one file.

What is real in this file:
- the intended state order for search, align, approach, stop, and arm pickup
- the servo target logging format for future calibration

What is mocked:
- bottle/can detection model output
- depth/distance updates
- chassis movement
- arm movement when DRY_RUN is True
"""

from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from pathlib import Path


# Keep True for presentation. Set False only after reviewing every servo target.
DRY_RUN = True

LOG_PATH = Path(__file__).with_name("redbull_servo_positions_log.csv")

# Approximate European small Red Bull can: 250 ml slim aluminium can.
CAN_VOLUME_ML = 250.0
CAN_DIAMETER_MM = 53.0
CAN_HEIGHT_MM = 133.0
CAN_EMPTY_MASS_G = 11.0
CAN_FULL_MASS_G_ESTIMATE = (261.0, 271.0)

# Coarse navigation thresholds. These are placeholders for real calibration.
CENTER_OFFSET_TOLERANCE_M = 0.08
STAGING_DISTANCE_M = 0.75


@dataclass
class CanObservation:
    """Predicted can data that will later come from detectNet + DepthNet."""

    found: bool
    distance_m: float
    horizontal_offset_m: float
    bbox_center_x: float
    bbox_height_ratio: float
    confidence: float


# Mock perception state. A real version should call bottle/can detectNet here.
mock_scan_count = 0
mock_distance_m = 5.0
mock_offset_m = 0.6

ttl_servo = None


def init_servo_if_needed():
    """Load JETANK servo API only when DRY_RUN is disabled."""
    global ttl_servo
    if DRY_RUN:
        return
    from SCSCtrl import TTLServo

    ttl_servo = TTLServo


def ensure_servo_log():
    """Create the external servo log used for later pose tuning."""
    if LOG_PATH.exists():
        return
    with LOG_PATH.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp", "step", "servo_id", "target", "speed", "note"])


def log_servo_target(step: str, servo_id: int, target: float, speed: int, note: str = ""):
    """Record every intended servo target, even in dry-run mode."""
    with LOG_PATH.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([f"{time.time():.3f}", step, servo_id, target, speed, note])


def move_servo(step: str, servo_id: int, target: float, speed: int, note: str = ""):
    """Send or print one servoAngleCtrl target."""
    print(f"[arm] {step}: servo={servo_id} target={target} speed={speed} {note}")
    log_servo_target(step, servo_id, target, speed, note)
    if ttl_servo is not None:
        ttl_servo.servoAngleCtrl(servo_id, target, 1, speed)
    time.sleep(0.25)


def apply_pose(step: str, targets: list[tuple[int, float]], speed: int, note: str = ""):
    """Apply a tiny placeholder pose and log all servo targets."""
    for servo_id, target in targets:
        move_servo(step, servo_id, target, speed, note)
    time.sleep(0.4)


def base_set_velocity(linear: float, angular: float, seconds: float):
    """Mock chassis movement. Replace with JetBot Robot later."""
    print(f"[base] linear={linear:+.2f} angular={angular:+.2f} for {seconds:.2f}s")
    time.sleep(seconds)
    base_stop()


def base_stop():
    print("[base] stop")


def print_model_prediction(label: str, observation: CanObservation):
    """Make it obvious that these are model-predicted values, currently mocked."""
    if not observation.found:
        print(f"[model:{label}] no Red Bull/can/bottle detection")
        return

    print(
        f"[model:{label}] MOCK bottle/can detection output: "
        f"found={observation.found}, conf={observation.confidence:.2f}, "
        f"distance={observation.distance_m:.2f}m, "
        f"horizontal_offset={observation.horizontal_offset_m:+.2f}m, "
        f"bbox_center_x={observation.bbox_center_x:.2f}, "
        f"bbox_height_ratio={observation.bbox_height_ratio:.2f}"
    )


def scan_for_can() -> CanObservation:
    """Mock the future bottle/can detection model during search."""
    global mock_scan_count
    mock_scan_count += 1

    if mock_scan_count < 3:
        observation = CanObservation(False, 0.0, 0.0, 0.0, 0.0, 0.0)
    else:
        # This block represents the untested future detectNet/depthNet result.
        observation = CanObservation(True, mock_distance_m, mock_offset_m, 0.62, 0.08, 0.80)

    print_model_prediction("scan", observation)
    return observation


def update_prediction_after_alignment() -> CanObservation:
    """Mock a new model prediction after the chassis turns toward the can."""
    global mock_offset_m
    mock_offset_m *= 0.35
    observation = CanObservation(True, mock_distance_m, mock_offset_m, 0.51, 0.08, 0.84)
    print_model_prediction("alignment_update", observation)
    return observation


def update_prediction_after_approach() -> CanObservation:
    """Mock a new model prediction after a short forward movement."""
    global mock_distance_m, mock_offset_m
    mock_distance_m = max(0.55, mock_distance_m - 0.55)
    mock_offset_m *= 0.8
    bbox_height_ratio = min(0.75, 0.08 + (5.0 - mock_distance_m) * 0.11)
    observation = CanObservation(True, mock_distance_m, mock_offset_m, 0.50, bbox_height_ratio, 0.86)
    print_model_prediction("approach_update", observation)
    return observation


def search_for_can() -> CanObservation:
    print("[state] SEARCH")
    while True:
        observation = scan_for_can()
        if observation.found:
            return observation

        print("[nav] no detection yet; rotate in place and scan again")
        base_set_velocity(linear=0.0, angular=0.18, seconds=0.35)


def align_to_can(observation: CanObservation) -> CanObservation:
    print("[state] ALIGN")
    while abs(observation.horizontal_offset_m) > CENTER_OFFSET_TOLERANCE_M:
        angular = -0.12 if observation.horizontal_offset_m > 0 else 0.12
        print(f"[nav] can offset={observation.horizontal_offset_m:+.2f}m; turn toward can")
        base_set_velocity(linear=0.0, angular=angular, seconds=0.25)
        observation = update_prediction_after_alignment()

    print("[nav] can roughly centered")
    base_stop()
    return observation


def approach_can(observation: CanObservation) -> CanObservation:
    print("[state] APPROACH")
    while observation.distance_m > STAGING_DISTANCE_M:
        print(
            f"[nav] can still far: distance={observation.distance_m:.2f}m, "
            f"bbox_height_ratio={observation.bbox_height_ratio:.2f}"
        )
        base_set_velocity(linear=0.16, angular=0.0, seconds=0.45)
        observation = update_prediction_after_approach()

        if abs(observation.horizontal_offset_m) > CENTER_OFFSET_TOLERANCE_M:
            observation = align_to_can(observation)

    print(f"[nav] staging distance reached: {observation.distance_m:.2f}m")
    base_stop()
    return observation


def pickup_can(observation: CanObservation):
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
    print(
        "[demo] can model: "
        f"{CAN_VOLUME_ML:.0f}ml, diameter={CAN_DIAMETER_MM:.0f}mm, "
        f"height={CAN_HEIGHT_MM:.0f}mm, empty={CAN_EMPTY_MASS_G:.1f}g, "
        f"full_estimate={CAN_FULL_MASS_G_ESTIMATE[0]:.0f}-{CAN_FULL_MASS_G_ESTIMATE[1]:.0f}g"
    )
    print("[demo] NOTE: bottle/can model predictions in this script are mocked, not yet tested.")

    observation = search_for_can()
    observation = align_to_can(observation)
    observation = approach_can(observation)
    pickup_can(observation)

    print(f"[demo] finished; servo log: {LOG_PATH}")


if __name__ == "__main__":
    main()
