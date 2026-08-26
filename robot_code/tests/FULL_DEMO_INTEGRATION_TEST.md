# Full Demo Integration Test

`tests/full_demo_integration_test.ipynb` is a thin board-side wrapper around production `DemoStateMachine`. It deliberately has no parameter widgets, preventing UI labels from drifting away from JSON keys.

## Iteration Loop

1. Edit and save `config.json` or `empirical_parameters.json`.
2. Click Reload Configuration in the notebook.
3. Run preflight or one FSM step.
4. Run the full demo when camera/model checks pass.
5. Inspect the notebook output and the timestamped file in `logs/`.

Reloading creates a fresh runtime from disk. Detector objects are reused during a runtime unless the reload-model option is selected. The notebook never writes configuration.

## Safety

Dry-run flags come from `config.json`. Start with all three enabled. Enable the camera, base, and arm separately as confidence grows. The arm should remain dry-run until `tuning_tools/arm_sequence_tuning.ipynb` has confirmed `safe_home` and the pickup sequence.

Every complete run executes cleanup. For interrupted notebook work, use Stop All and Release Camera before restarting a kernel. If the Python process is terminated outside its cleanup path, release the camera before opening another notebook.

## Expected Results

- A dry-run reaches `DONE` with one simulated pickup and release.
- A real preflight reports a camera frame plus can, tag, and optional depth observations.
- A failed real mission reports the final state and reason, stops the base, attempts `safe_home`, and closes the camera.
- Logs use `[fsm]`, `[mission]`, `[approach]`, `[can]`, `[tag]`, `[depth]`, `[arm]`, `[base]`, and `[tangentbug]` prefixes. `[mission] completed_pickups=N` reports mission progress after a successful pickup.
