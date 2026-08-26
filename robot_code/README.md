# JetTank Can Pickup Demo

This directory is the self-contained delivery package for the JetTank demo. It has one production state machine, one command-line entry point, and two active JSON configuration files.

## Runtime Flow

```text
INIT -> PLAN -> SEARCH -> ALIGN -> APPROACH -> FINAL_VERIFY
     -> PICKUP -> PLAN -> SEARCH BIN -> ALIGN BIN -> APPROACH BIN
     -> RELEASE -> DONE
```

`APPROACH` can be interrupted by the optional scripted or depth-profile TangentBug avoidance strategy. A failed search, alignment, approach, or final verification enters the bounded retry path; exhausted retries end in `FAILED`. Every terminal path calls `stop_all()`.

## Configuration

- `config.json`: project paths, dry-run switches, feature switches, score policy, retry policy, and avoidance strategy.
- `empirical_parameters.json`: camera dimensions, model settings, thresholds, motion speeds, pulse lengths, ROIs, and arm poses.
- `legacy_params/old_demo_params_reference.json`: the only retained flat legacy configuration. It is documentation only and is never loaded.

The merge order is:

```text
empirical_parameters.json -> config.json -> command-line overrides
```

## Board Workflow

From the project root on the Jetson:

```bash
python3 tests/board_first_checks.py
python3 run_demo.py --validate-only --no-log-file
python3 run_demo.py --dry-run --no-log-file
```

Use the notebooks for incremental checks:

- `tests/camera_network_diagnostics.ipynb`: camera, can model, AprilTag, depth, overlays, and model reuse.
- `tests/full_demo_integration_test.ipynb`: production `DemoRuntime`, step-by-step execution, preflight, and full-run logging.
- `tuning_tools/arm_sequence_tuning.ipynb`: direct editing/testing of the `arm` section in `empirical_parameters.json`.

Real hardware can be enabled together or independently:

```bash
python3 run_demo.py --real
python3 run_demo.py --camera-real --base-real
python3 run_demo.py --camera-real --base-real --arm-real --avoidance tangentbug_depth
```

Hardware is dry-run by default. Base actions are short pulses and `stop_all()` stops the base, returns the arm to `safe_home`, and releases the camera.

## Models

Can detection uses the standard jetson-inference `detectNet` API directly:

```text
assets/models/detectnet_native_can/can_ssd_mobilenet_v1.onnx
assets/models/detectnet_native_can/labels.txt
input_0 -> scores + boxes
confidence=0.20, clustering=0.30
```

Run the first Nano image smoke test before enabling base or arm motion:

```bash
python3 tests/validate_can_detectnet_image.py assets/samples/can1.jpg \
  --output diagnostic_outputs/detectnet_native_can.jpg
```

There is no runtime backend switch or custom TFOD/PyCUDA post-processing path in this branch. Can parameters are stored directly under `detectors.can`.

AprilTag detection uses `pupil_apriltags` with `tag36h11`; it does not require a learned model. The printable tag is `assets/bin_apriltag_36h11_id_0.png`.

## Logs And References

Runtime logs are written under `logs/` and ignored by Git. Two curated legacy examples are retained under `legacy_params/logs/` to show a successful and a failed earlier run. See `PROJECT_STRUCTURE.md`, `FSM_ARCHITECTURE.md`, and `tests/NOTEBOOK_PARAMETER_GUIDE.md` for detailed ownership and testing guidance.
