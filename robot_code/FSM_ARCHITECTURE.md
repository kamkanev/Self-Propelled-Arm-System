# Demo FSM Architecture

## Mission Model

`DemoStateMachine` is the only production coordinator. `MissionContext` stores the current target, grabbed state, completed pickup count, retry bookkeeping, state timing, and last visual observation. A typed `VagueMap` separately owns approximate robot pose, mapped cans, patrol progress, and the bin docking poses.

Responsibilities are separated as follows:

- `demo_core/fsm_types.py` defines mission states, events, target types, and context bookkeeping.
- `demo_core/state_machine.py` owns transitions, handlers, finalization, retries, and cleanup.
- `demo_core/navigation.py` owns target search, alignment, approach, verification, and optional side docking.
- `demo_core/vague_map.py` owns command odometry, approximate can memory, patrol progression, and coarse point navigation.
- `demo_core/perception.py` converts can, AprilTag, and depth results into shared observations.
- `demo_core/robot_control.py` translates navigation requests into base and servo commands.

The nominal transition graph is:

```text
IDLE -> INITIALIZING -> PLANNING
PLANNING -> SEARCHING | PATROLLING | MAP_NAVIGATING | DONE
SEARCHING -> ALIGNING | PLANNING | INTERMEDIATE
PATROLLING -> ALIGNING | PLANNING | INTERMEDIATE
MAP_NAVIGATING -> ALIGNING | SEARCHING | PLANNING | INTERMEDIATE
ALIGNING -> APPROACHING | SEARCHING | INTERMEDIATE
APPROACHING -> FINAL_VERIFY | SEARCHING | AVOIDING | INTERMEDIATE
FINAL_VERIFY -> FINALIZING | BIN_SIDE_DOCKING | SEARCHING | INTERMEDIATE
BIN_SIDE_DOCKING -> FINALIZING | INTERMEDIATE
AVOIDING -> VERIFY_TARGET | INTERMEDIATE
FINALIZING -> PLANNING
INTERMEDIATE -> PLANNING | FAILED
```

A successful pickup sets `grabbed=true` and increments `completed_pickups`. The bin docking pose then becomes the coarse map destination. After release, planning prefers the nearest mapped can before resuming the interrupted patrol waypoint. A positive `runtime.max_pickups` is an early completion limit; zero delegates completion to patrol progress and mapped-can memory.

## Vague Map Data Flow

Base motion reports the effective, scaled command to `CommandOdometry`. Straight movement updates position using calibrated meters per speed-second and a slip factor; turns update and normalize heading. These estimates are intentionally approximate.

Every map-navigation update checks visual detection before issuing motion. A visible target immediately stops map travel and enters visual alignment. Reaching a map coordinate only starts a visual search; it never claims that the physical target has been reached.

While carrying a can toward the bin, the runtime can map incidental can detections. One depth field is reused for all candidates in the frame, upper-image candidates are rejected, and accepted world coordinates are bounds-checked and merged by radius.

## Visual Navigation

Can and bin navigation share four stages:

1. `SEARCHING` rotates until the target is accepted or the state times out.
2. `ALIGNING` uses normalized horizontal error and stops immediately if tracking is lost.
3. `APPROACHING` reacquires each frame, steers when needed, and stops on the configured bounding-box-height or depth condition.
4. `FINAL_VERIFY` requires stable frames under tighter alignment rules.

Search, alignment, steering, and map heading correction use continuous commands where configured. Visual updates, target-loss limits, and timeouts are responsible for stopping them. Deterministic recovery maneuvers remain time-bounded.

The optional experimental side-docking path preserves normal front-facing bin navigation, then moves the arm to `side_view_grabbing`, waits for the servos to settle, performs the supportive base turn, and uses AprilTag geometry for base-only correction. Release uses `side_view_release_low`, followed by `safe_home` and the side-parking map-pose reset. Failure restores the carry view and reverses the supportive heading change before retry handling.

## Avoidance

Avoidance is selected by `avoidance.strategy`:

- `disabled` never interrupts approach.
- `scripted` uses fixed, time-bounded bypass motion.
- `tangentbug_depth` evaluates a smoothed depth-column profile, free-space gaps, and depth discontinuities, then emits one incremental correction at a time.

The planner scores candidate headings using target direction and local clearance. Avoidance is independent of `VagueMapNavigator`.

## Finalization And Cleanup

Can finalization runs the configured arm-down, optional push, grab, carry, and optional verification sequence. Bin finalization normally runs `release -> safe_home`; the experimental side-docking path uses its dedicated release pose first.

Timeouts enter `INTERMEDIATE` and retry up to `runtime.retry_limit`. Unexpected exceptions enter `FAILED`. `run()` always invokes cleanup unless ownership was explicitly retained by a caller, and repeated cleanup calls are safe.

## Public API

```python
from demo_core import DemoDiagnostics, DemoStateMachine, load_config

config = load_config()
diagnostics = DemoDiagnostics(config)
state_machine = DemoStateMachine(config)

state_machine.step_once()
state_machine.run()
state_machine.stop_all()
state_machine.release_camera()
```
