# Project Structure

```text
robot_code/
  config.json
  empirical_parameters.json
  run_demo.py
  demo_core/
    config.py
    diagnostics.py
    depth_vision.py
    fsm_types.py
    navigation.py
    perception.py
    robot_control.py
    state_machine.py
    tangentbug.py
    vague_map.py
    logging_utils.py
  assets/
    bin_apriltag_36h11_id_0.png
    models/detectnet_native_can/
    samples/can1.jpg
  tests/
  tuning_tools/
  logs/
  legacy_params/
```

## Runtime Ownership

- `run_demo.py` is the only complete production entry point.
- `state_machine.py` owns mission coordination, retries, pickup and release sequencing, and cleanup.
- `fsm_types.py` defines states, events, target types, and shared mission context.
- `navigation.py` owns visual navigation and optional experimental side docking.
- `vague_map.py` owns approximate map data, command odometry, mapped-can merging, patrol progress, and coarse navigation.
- `perception.py` exposes can, AprilTag, and depth observations.
- `depth_vision.py` owns the JetBot camera and DepthNet lifecycle.
- `robot_control.py` owns scaled base commands and servo operations.
- `tangentbug.py` owns optional local depth-obstacle planning.
- `config.py` merges and validates the two active JSON configuration files.

`tests/` contains diagnostics and lightweight verification. `tuning_tools/` contains non-production calibration utilities. Neither directory provides an alternative state machine or runtime entry point.

The can detector uses the standard jetson-inference `detectNet` interface and exposes the observation contract consumed by navigation. The runtime does not include a second can-inference backend.

## Generated And Historical Files

Runtime logs, diagnostic output, TensorRT engine files, notebook checkpoints, and Python bytecode are generated artifacts and must not be included in runtime synchronization commits.

`legacy_params/` and `legacy_prototype/` are historical references. Production code must not import from them.
