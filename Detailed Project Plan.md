# Detailed Project Plan

## 1. Minimum Deliverable Project Version

1. Start the program.
2. Detect the crumpled paper ball using the onboard camera.
3. Decide whether the target is left, center, or right in the camera image.
4. Turn left, turn right, or move forward based on the target position.
5. Stop at the calibrated grasping distance.
6. Pick up the paper ball using a predefined arm and gripper sequence.
7. Move to a marked drop-off area with a clear visual marker.
8. Release the paper ball at the calibrated release position.
9. Stop and return to a safe state.

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
|-- best.pt
|-- config.yaml
|-- train_paper.py
|-- test_detection.py
|-- paper_detect/
|-- perception/
|   |-- camera.py
|   |-- paper_detector.py
|   `-- bin_detector.py
|-- control/
|   |-- base_controller.py
|   |-- arm_controller.py
|   |-- safety.py
|   `-- approach.py
|-- workflow/
|   `-- state_machine.py
|-- calibration/
|   |-- motion_params.yaml
|   |-- arm_positions.yaml
|   `-- vision_params.yaml
|-- scripts/
|   |-- test_camera.py
|   |-- test_base.py
|   |-- test_arm.py
|   |-- test_paper_detector.py
|   |-- test_approach_bin.py
|   `-- run_robot.py
`-- docs/
    |-- interface_contract.md
    |-- testing_log.md
```

## 5. Detailed Task List

### 5.1 Vision Software Tasks

VS-01 YOLO detection:

- Run `python test_detection.py`.

VS-02 Wrap the paper detector:

- Create `perception/paper_detector.py`.
- Implement `detect_paper(frame)`.

VS-03 Define direction and distance states:

- Use bbox center to determine left, center, or right.
- Use bbox area or height to determine far, near, or grasp_range after real distance calibration.
- Save vision thresholds and calibration values to `calibration/vision_params.yaml`.

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
