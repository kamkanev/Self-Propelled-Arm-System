# Detailed Project Plan

## 1. Minimum Deliverable Version

1. Start the program.
2. Detect a crumpled paper ball (?or easy-to-grasp object like colored foam cube) using the onboard camera.
3. Decide whether the target is left, center, or right in the camera image.
4. Turn left, turn right, or move forward based on the target position.
5. Stop at the calibrated grasping distance, wait briefly after stopping.
6. Perform a multi-frame check to confirm that the target is still inside the calibrated grasping range.
7. Pick up the target using a predefined arm and gripper sequence.
8. Verify whether the grasp likely succeeded.
9. Move to a marked drop-off area with a clear visual marker (?colored/AprilTag) and release.
10. Stop and return to a safe state.


## 2. Current Missing Parts

- JETANK chassis, arm, and gripper control.
- YOLO detection code results to robot states.
- Motion decision logic based on onboard camera output.
- Drop-off bin marker detection using a large color marker or visual tag.
- Complete implementation of the state machine.

## 3. Team Roles

Two groups: Software and Hardware.

- The software group decides what the robot should do and when.
- The hardware group makes the robot execute physical actions reliably.

Both groups must follow a clear interface contract.

### 3.1 Vision Software Group

Responsibilities:

- Run and validate the detection code with the onboard camera.
- Wrap the paper detector into a reusable module.
- Convert detection boxes into direction and distance states.
- Implement drop-off marker detection for a target area.
- Implement the state machine and task flow runs.
- Provide high-level decisions and commands.

Deliverables:

- `perception/paper_detector.py`
- `perception/bin_detector.py`
- `workflow/state_machine.py`
- `scripts/test_paper_detector.py`
- `scripts/run_robot.py`
- `calibration/vision_params.yaml`
- `docs/interface_contract.md`
- `docs/testing_log.md`

### 3.2 Hardware Group: Chassis Part

Responsibilities:

- Implement chassis control.
- Calibrate forward motion, turning, and stopping.
- Execute movement commands from the software decision layer.
- Stop the robot when an error occurs, when the target is lost, or when the camera frame cannot be read.
- Ensure the chassis does not move while the arm is moving.
- Verify the robot can approach paper and drop-off bin targets at safe speed.

Deliverables:

- `control/base_controller.py`
- `control/safety.py`
- `calibration/motion_params.yaml`
- `scripts/test_base.py`
- `scripts/test_approach_bin.py`

### 3.3 Hardware Group: Arm and Gripper Part

Responsibilities:

- Implement gripper open and close actions.
- Calibrate servo IDs and safe angle ranges before running full arm sequences.
- Calibrate home, pre-grasp, grasp, lift, carry, and release poses.
- Complete fixed-position grasping after the robot stops at the calibrated grasping distance.
- Complete the release action without hitting the camera, chassis, or drop-off bin.

Deliverables:

- `control/arm_controller.py`
- `calibration/arm_positions.yaml`
- `scripts/test_arm.py`


## 4. Module Interface and Repository Structure

```text
Self-Propelled-Arm-System/
|-- README.md
|-- requirements.txt
|-- config.yaml
|-- models/
|   `-- best.pt
|-- perception/
|   |-- camera.py
|   |-- paper_detector.py
|   |-- bin_detector.py
|   `-- filters.py
|-- control/
|   |-- base_controller.py
|   |-- arm_controller.py
|   |-- gripper_controller.py
|   |-- safety.py
|   `-- watchdog.py
|-- workflow/
|   |-- state_machine.py
|   `-- state_defs.py
|-- calibration/
|   |-- motion_params.yaml
|   |-- arm_positions.yaml
|   |-- vision_params.yaml
|   `-- bin_params.yaml
|-- scripts/
|   |-- test_camera.py
|   |-- test_base.py
|   |-- test_arm.py
|   |-- test_paper_detector.py
|   |-- test_bin_detector.py
|   |-- test_grasp_sequence.py
|   |-- test_full_v0.py
|   |-- test_full_v1.py
|   `-- run_robot.py
|-- docs/
|   |-- interface_contract.md
|   |-- calibration_guide.md
|   |-- testing_log.md
|   |-- failure_cases.md
|   `-- demo_protocol.md
`-- training/
    |-- train_paper.py
    `-- test_detection.py
```

## 5. Detailed Task List

### 5.1 Vision Software Tasks

VS-01 YOLO detection:

- Run `python test_detection.py`.
Validate that the model works with the onboard camera.

VS-02 Wrap the paper detector:

- Create `perception/paper_detector.py`.
- Implement `detect_paper(frame)`.
The function should return a detection object or None.

VS-03 Define direction and distance states:

- Use bbox center to determine left, center, or right.
- Use bbox area or height to determine far, near, or grasp_range after real distance calibration.
- Save vision thresholds and calibration values to `calibration/vision_params.yaml`.
The exact values must be calibrated on the real robot.

VS-04 Implement bin marker detection:

- Create `perception/bin_detector.py` for a large color marker or simple visual tag.

VS-05 Implement the state machine:

- Create `workflow/state_machine.py`.
- Include the main flow for searching, aligning, approaching, grasping, moving to the drop-off bin, releasing, and safe stopping.

VS-06 Prepare testing and run scripts:

- `scripts/test_camera.py`.
- `scripts/test_paper_detector.py`.
- `scripts/run_robot.py`.
- Record software and hardware test results in `docs/testing_log.md`.

### 5.2 Hardware Tasks

HW-01 Hardware check:

- Test camera.
- Test chassis.
- Test arm.
- Test gripper.
- Check battery.

HW-02 Chassis control:

- Create `control/base_controller.py`.
- Implement forward, backward, left turn, right turn, stop, and emergency stop.

HW-03 Arm control:

- Create `control/arm_controller.py`.
- Implement home, open_gripper, close_gripper, pre_grasp, grasp, lift, carry, and release.

HW-04 Arm calibration:

- Record servo IDs.
- Record safe angles and avoid poses that collide with the camera or chassis.
- Save poses to `calibration/arm_positions.yaml`.

HW-05 Chassis motion calibration:

- Test forward movement.
- Test turning.
- Test stopping response.
- Save parameters to `calibration/motion_params.yaml`.

HW-06 Visual approach execution:

- Execute movement commands from the software decision layer.
- Tune movement speed and stop behavior on the real floor.
- Stop when the target is lost or when the camera frame cannot be read.

HW-07 Fixed-position grasp:

- Calibrate paper ball placement distance.
- Tune gripper and arm poses.
- Confirm that the paper ball can be grasped after the robot stops at the calibrated distance.

HW-08 Align approach stop with grasp range:

- Tune `grasp_range` with the software group.
- Ensure the gripper can reach the paper after the robot stops, then update the bbox threshold.

HW-09 Release action:

- Calibrate release_pose for the marked drop-off bin or fixed drop-off area.
- Test drop-off success rate without the arm touching the bin.

HW-10 Safety rules:

- Stop on error using `try/finally` or equivalent cleanup.
- Stop chassis before any arm movement.
- Keep the arm in carry pose before moving toward the drop-off bin.
- Stop the chassis before releasing the paper ball.



## 6. State Machine Pseudocode

```python

state = "SEARCH_TARGET"
retry_count = 0
last_state_change_time = time.time()

try:
    while True:
        frame = camera.read()

        # Global safety check: stop immediately if camera fails
        if frame is None:
            base.emergency_stop()
            state = "ERROR"

        # Global timeout check: avoid getting stuck in one state forever
        if state_timeout_exceeded(state, last_state_change_time):
            base.stop()
            retry_count += 1
            state = recover_from_timeout(state, retry_count)
            last_state_change_time = time.time()
            continue

        # 1. Search for the target object
        if state == "SEARCH_TARGET":
            target = target_detector.detect(frame)

            if target is None:
                # Rotate slowly until the target appears
                base.set_velocity(0.0, SEARCH_TURN_SPEED)
            else:
                base.stop()
                state = "APPROACH_TARGET"
                last_state_change_time = time.time()

        # 2. Align with and approach the target
        elif state == "APPROACH_TARGET":
            target = target_detector.detect(frame)

            if target is None:
                # If the target is lost, stop and search again
                base.stop()
                state = "SEARCH_TARGET"
                last_state_change_time = time.time()
                continue

            center_x = target.bbox_center_x
            bbox_height = target.bbox_height
            error_x = center_x - IMAGE_CENTER_X

            if error_x < -CENTER_TOLERANCE:
                # Target is on the left side of the image
                base.set_velocity(0.0, TURN_SPEED)

            elif error_x > CENTER_TOLERANCE:
                # Target is on the right side of the image
                base.set_velocity(0.0, -TURN_SPEED)

            elif bbox_height < GRASP_BBOX_HEIGHT_MIN:
                # Target is centered but still too far away
                base.set_velocity(APPROACH_SPEED, 0.0)

            elif bbox_height > GRASP_BBOX_HEIGHT_MAX:
                # Target is too close, back up slightly
                base.set_velocity(-BACKUP_SPEED, 0.0)

            else:
                # Target is centered and within the calibrated grasping range
                base.stop()
                sleep(0.5)
                state = "FINAL_CHECK_TARGET"
                last_state_change_time = time.time()

        # 3. Final multi-frame check before grasping
        elif state == "FINAL_CHECK_TARGET":
            base.stop()
            stable_count = 0

            for _ in range(FINAL_CHECK_FRAMES):
                frame = camera.read()
                target = target_detector.detect(frame)

                if target is not None:
                    error_x = target.bbox_center_x - IMAGE_CENTER_X
                    bbox_height = target.bbox_height

                    if (
                        target.confidence >= CONF_THRESHOLD
                        and abs(error_x) < FINAL_CENTER_TOLERANCE
                        and GRASP_BBOX_HEIGHT_MIN <= bbox_height <= GRASP_BBOX_HEIGHT_MAX
                    ):
                        stable_count += 1

                sleep(0.05)

            if stable_count >= REQUIRED_STABLE_FRAMES:
                state = "GRASP_TARGET"
            else:
                state = "APPROACH_TARGET"

            last_state_change_time = time.time()

        # 4. Execute predefined arm and gripper grasping sequence
        elif state == "GRASP_TARGET":
            base.stop()

            arm.pre_grasp()
            gripper.open()

            arm.grasp()
            gripper.close()
            sleep(GRIPPER_SETTLE_TIME)

            arm.lift()
            arm.carry()

            state = "VERIFY_GRASP"
            last_state_change_time = time.time()

        # 5. Verify whether grasp likely succeeded
        elif state == "VERIFY_GRASP":
            frame = camera.read()
            target = target_detector.detect(frame)

            if grasp_likely_successful(target):
                state = "SEARCH_DROPOFF"
            else:
                retry_count += 1

                if retry_count <= MAX_GRASP_RETRIES:
                    state = "FINAL_CHECK_TARGET"
                else:
                    state = "ERROR"

            last_state_change_time = time.time()

        # 6. Search for the marked drop-off area
        elif state == "SEARCH_DROPOFF":
            marker = dropoff_detector.detect(frame)

            if marker is None:
                # Rotate slowly until the drop-off marker appears
                base.set_velocity(0.0, SEARCH_TURN_SPEED)
            else:
                base.stop()
                state = "APPROACH_DROPOFF"
                last_state_change_time = time.time()

        # 7. Align with and approach the drop-off area
        elif state == "APPROACH_DROPOFF":
            marker = dropoff_detector.detect(frame)

            if marker is None:
                # If the marker is lost, stop and search again
                base.stop()
                state = "SEARCH_DROPOFF"
                last_state_change_time = time.time()
                continue

            center_x = marker.bbox_center_x
            bbox_height = marker.bbox_height
            error_x = center_x - IMAGE_CENTER_X

            if error_x < -DROPOFF_CENTER_TOLERANCE:
                # Drop-off marker is on the left side of the image
                base.set_velocity(0.0, TURN_SPEED)

            elif error_x > DROPOFF_CENTER_TOLERANCE:
                # Drop-off marker is on the right side of the image
                base.set_velocity(0.0, -TURN_SPEED)

            elif bbox_height < RELEASE_BBOX_HEIGHT_MIN:
                # Marker is centered but still too far away
                base.set_velocity(DROPOFF_APPROACH_SPEED, 0.0)

            elif bbox_height > RELEASE_BBOX_HEIGHT_MAX:
                # Marker is too close, back up slightly
                base.set_velocity(-BACKUP_SPEED, 0.0)

            else:
                # Marker is centered and within the calibrated release range
                base.stop()
                sleep(0.5)
                state = "FINAL_CHECK_DROPOFF"
                last_state_change_time = time.time()

        # 8. Final multi-frame check before releasing
        elif state == "FINAL_CHECK_DROPOFF":
            base.stop()
            stable_count = 0

            for _ in range(FINAL_DROPOFF_CHECK_FRAMES):
                frame = camera.read()
                marker = dropoff_detector.detect(frame)

                if marker is not None:
                    error_x = marker.bbox_center_x - IMAGE_CENTER_X
                    bbox_height = marker.bbox_height

                    if (
                        abs(error_x) < FINAL_DROPOFF_CENTER_TOLERANCE
                        and RELEASE_BBOX_HEIGHT_MIN <= bbox_height <= RELEASE_BBOX_HEIGHT_MAX
                    ):
                        stable_count += 1

                sleep(0.05)

            if stable_count >= REQUIRED_STABLE_DROPOFF_FRAMES:
                state = "RELEASE_TARGET"
            else:
                state = "APPROACH_DROPOFF"

            last_state_change_time = time.time()

        # 9. Release the target at the calibrated drop-off position
        elif state == "RELEASE_TARGET":
            base.stop()

            arm.release_pose()
            gripper.open()
            sleep(RELEASE_SETTLE_TIME)

            arm.home()
            state = "SAFE_STOP"
            last_state_change_time = time.time()

        # 10. Safe stop after successful task completion
        elif state == "SAFE_STOP":
            base.stop()
            arm.safe_pose()
            break

        # Error handling
        elif state == "ERROR":
            base.emergency_stop()
            arm.safe_pose()
            break

finally:
    base.emergency_stop()
    arm.safe_pose()
```
