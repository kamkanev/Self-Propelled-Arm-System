# JetTank Can Pickup Demo

`robot_code` is the self-contained runtime delivery for the JetTank can-pickup demo. It contains one production state machine, one command-line entry point, and two active JSON configuration files.

## Runtime Flow

```text
INITIALIZE -> PLAN -> SEARCH CAN -> PATROL WHEN NEEDED
           -> ALIGN -> APPROACH -> FINAL VERIFY -> PICK UP
           -> MAP NAVIGATE TO BIN -> VISUAL BIN DOCK -> RELEASE
           -> NEXT MAPPED CAN OR PATROL WAYPOINT -> DONE
```

The vague map uses command-based open-loop odometry for coarse travel. Visual detection is attempted before map motion and takes control as soon as the requested target is visible. Obstacle avoidance remains an independent approach-time behavior; the vague map does not plan around obstacles.

## Configuration

- `config.json` owns project paths, dry-run switches, mission policy, feature switches, retry limits, and the avoidance strategy.
- `empirical_parameters.json` owns camera and detector settings, named base-speed profiles, per-direction command scales, navigation thresholds, vague-map calibration, and arm poses.
- `legacy_params/old_demo_params_reference.json` is historical documentation and is never loaded by production code.

Configuration is merged in this order:

```text
empirical_parameters.json -> config.json -> command-line overrides
```

Base motion settings reference symbolic profiles such as `linear.fast` and `turn.slow`. `base_motion_command_scales` applies a separate positive multiplier to forward, backward, left, and right commands. All four multipliers default to `1.0`, so they do not change current behavior until calibrated.

## Validation And Execution

From `robot_code` on the Jetson:

```bash
python3 tests/board_first_checks.py
python3 run_demo.py --validate-only --no-log-file
python3 run_demo.py --dry-run --no-log-file
```

Hardware is dry-run by default. Enable only the required devices:

```bash
python3 run_demo.py --real
python3 run_demo.py --camera-real --base-real
python3 run_demo.py --camera-real --base-real --arm-real --avoidance tangentbug_depth
```

Continuous motion is bounded by visual feedback, target-loss limits, arrival thresholds, and state timeouts. `stop_all()` stops the base, requests cooperative arm cancellation, returns the arm to `safe_home`, and releases the camera.

## Perception And Avoidance

Can detection uses the jetson-inference `detectNet` API with the bundled ONNX model and labels under `assets/models/detectnet_native_can/`. Bin detection uses `pupil_apriltags` with the `tag36h11` family and the configured tag ID.

`avoidance.strategy` supports `disabled`, `scripted`, and `tangentbug_depth`. The depth planner extracts obstacle contours, filters floor and wide-wall geometry, selects a visible tangent side, and emits incremental motion decisions. It is a local reactive planner, not metric localization or a complete global TangentBug implementation.

## References

See `FSM_ARCHITECTURE.md` for state and data flow, and `PROJECT_STRUCTURE.md` for runtime ownership. Runtime logs are written under `logs/` and are not part of the delivered source changes.
