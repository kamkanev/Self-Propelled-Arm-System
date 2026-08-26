# Notebook And Parameter Guide

## Two Active JSON Files

`config.json` contains operational choices: paths, hardware dry-run flags, retry policy, detector enablement, depth enablement, and avoidance strategy.

Mission completion uses `runtime.max_pickups`. The current count is held in memory as `MissionContext.completed_pickups`; it is not an empirical parameter and is reset to zero for a new runtime.

`empirical_parameters.json` contains values learned through testing: speeds, pulse durations, timeouts, tolerances, detector thresholds, stop thresholds, ROIs, servo orders, and poses.

Saving a notebook itself does not save parameters. Parameters change on disk only when the arm tuning notebook explicitly invokes its save action, or when the JSON file is edited and saved directly.

## Camera And Network Diagnostics

Open `tests/camera_network_diagnostics.ipynb`. It uses `DemoDiagnostics`; it does not contain a second camera/model implementation.

Recommended order:

1. Reload configuration.
2. Start camera and capture a frame.
3. Load the can detector and test one frame.
4. Load the AprilTag detector and test one frame.
5. Test depth if `camera.depth_enabled` is true.
6. Start the low-rate overlay for visual center/bounding-box checks.
7. Release camera before restarting the kernel or opening another camera notebook.

The reload-model checkbox controls whether the cached detector object is discarded. Leave it off during ordinary tuning to avoid repeated detectNet initialization.

Can model paths, confidence thresholds, and clustering are stored directly under `detectors.can` in `empirical_parameters.json`.

## Full Demo Integration

Open `tests/full_demo_integration_test.ipynb`. It creates the same `DemoStateMachine` used by `run_demo.py`.

- Reload configuration reads both JSON files again and creates a fresh runtime.
- Preflight starts the camera and checks depth/can/tag through `DemoDiagnostics`.
- Step executes one FSM transition for controlled inspection.
- Run executes the complete mission.
- Stop requests a safe stop; Release Camera closes the shared camera path.

Logs are mirrored to a timestamped file under `logs/`. The notebook does not expose sliders and does not write either JSON file.

## Arm Sequence Tuning

Open `tuning_tools/arm_sequence_tuning.ipynb`. It directly reads and writes only the `arm` section of `empirical_parameters.json`.

Pose names:

| Pose | Meaning |
| --- | --- |
| `safe_home` | collision-conscious initial/final pose and camera orientation |
| `arm_down` | lower the claw to pickup height |
| `grab` | close the gripper; all five servo values are explicit |
| `carry` | lift and retain the object for base motion |
| `release` | position and open the claw at the bin |

The base push is not an arm pose. It uses `arm.push.speed`, `arm.push.seconds`, and `arm.push.post_lock_seconds`.

Use dry-run first. A Show action reads and executes the saved pose without writing. Apply executes the current in-memory values without writing. Save writes the selected pose to `empirical_parameters.json`; different pose buttons save only their corresponding pose object.

## Navigation Parameters

Each target has `search`, `align`, `approach`, and optional `near_align` sections under `navigation`.

- Search: `direction`, `speed`, `pulse_seconds`, `timeout_seconds`.
- Align: `speed`, `pulse_seconds`, `tolerance_norm`, `max_steps`, `lost_frame_limit`.
- Approach: forward and steering speeds/pulses, timeout, pulse limits, lost-frame limit, and `stop`.
- Stop mode `bbox_height`: stop when normalized box height is greater than or equal to the threshold.
- Stop mode `depth`: stop when `raw_depth * depth_scale` is less than or equal to the threshold.
- Near align: a tighter post-approach center check before arm/bin finalization.

Edit the JSON, save it, then reload configuration in the notebook before the next test. Existing in-memory objects do not automatically notice disk changes.
