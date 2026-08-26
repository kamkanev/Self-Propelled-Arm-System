# Demo FSM Architecture

## Mission Model

`DemoRuntime` is the only production coordinator. It owns a `MissionContext` containing the current target, vague target map, grabbed state, completed pickup count, retries, state timing, and last observation.

The FSM definition and implementation are deliberately separated:

- `demo_core/fsm_types.py` defines `MissionState`, `MissionEvent`, `TargetType`, and `MissionContext`. It contains mission data and bookkeeping but no transition table.
- `demo_core/state_machine.py` defines `DemoRuntime`, the transition table, state handlers, finalization sequence, retries, and cleanup.
- `demo_core/navigation.py` implements target search, alignment, approach, and final verification actions requested by the state machine.
- `demo_core/robot_control.py` translates those actions into base and servo commands.
- `demo_core/depth_vision.py` manages the JetBot camera and depthNet lifecycle. `demo_core/perception.py` converts camera/model results into common target observations.

The nominal transition chain is:

```text
IDLE -> INITIALIZING -> PLANNING
PLANNING -> VERIFY_TARGET (cached target) | SEARCHING (target required) | DONE
VERIFY_TARGET -> ALIGNING | SEARCHING
SEARCHING -> ALIGNING | INTERMEDIATE
ALIGNING -> APPROACHING | SEARCHING | INTERMEDIATE
APPROACHING -> FINAL_VERIFY | SEARCHING | AVOIDING | INTERMEDIATE
FINAL_VERIFY -> FINALIZING | SEARCHING | INTERMEDIATE
AVOIDING -> VERIFY_TARGET | INTERMEDIATE
FINALIZING -> PLANNING
INTERMEDIATE -> PLANNING | FAILED
```

The first mission target is `CAN`. A successful pickup sets `grabbed=true`, increments `completed_pickups`, and changes the planned target to `BIN`. A successful release clears `grabbed`. Once `max_pickups` is reached, planning enters `DONE`.

`completed_pickups` is the only mission progress counter. There is no separate score flag or score value. Detector confidence remains part of an observation and must not be interpreted as mission progress.

## Navigation Contract

Can and bin navigation use the same four-stage contract:

1. `SEARCHING`: rotate in short pulses until the target is accepted or the state timeout expires.
2. `ALIGNING`: use normalized horizontal center error; turn toward the observed target until inside tolerance.
3. `APPROACHING`: reacquire every frame, steer when necessary, and stop using the configured `bbox_height` or `depth` threshold.
4. `FINAL_VERIFY`: repeat a tighter alignment for the required number of stable frames.

Can acceptance includes confidence thresholds. AprilTag acceptance is based on the configured tag ID. Both targets expose the same observation fields such as `found`, `bbox`, `center_x`, `error_x`, and normalized box height.

The navigator consumes a stable can observation contract and does not handle inference details. `CanDetector` is the sole adapter from native jetson-inference detections to that contract.

## Finalization

Can finalization is deterministic:

```text
safe_home (during initialization)
arm_down
wait for servo positions
optional base push
grab
carry
optional depth verification
```

Bin finalization runs `release -> safe_home`. Mechanical-arm tuning belongs in `tuning_tools/arm_sequence_tuning.ipynb`; the FSM only consumes the resulting empirical parameters.

## Avoidance

Avoidance is configured by `avoidance.strategy`:

- `disabled`: approach never enters avoidance.
- `scripted`: fixed turn, forward, and rejoin pulses, then target reacquisition.
- `tangentbug_depth`: converts a depth profile into free-space gaps, chooses a local heading using target direction and clearance, and emits one incremental motion action. Optional overlays are written under `logs/`.

The planner is deliberately incremental. It does not claim metric localization or a complete global TangentBug implementation.

## Failure And Cleanup

Search/alignment/approach timeouts enter `INTERMEDIATE`. The failed target memory is cleared and the mission retries up to `runtime.retry_limit`; then it enters `FAILED`. Unexpected exceptions also enter `FAILED`.

`run()` always invokes `stop_all()` in `finally`, which stops the base, attempts `safe_home`, and releases the camera. Notebook users can also call `stop_all()` and `release_camera()` directly.

## Public API

```python
from demo_core import DemoDiagnostics, DemoRuntime, load_config

config = load_config()
diagnostics = DemoDiagnostics(config)
runtime = DemoRuntime(config)

runtime.step_once()
runtime.run()
runtime.stop_all()
runtime.release_camera()
```

Configuration is merged as `empirical_parameters.json -> config.json -> overrides`. The old flat reference is not adapted or loaded.
