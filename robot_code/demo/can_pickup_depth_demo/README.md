# Can Pickup Depth Demo

Purpose: run a simple end-to-end pickup demo for today's test.

Scope:

- Object recognition is mocked by manual placement.
- Path planning is mocked by optional hardcoded movement steps.
- DepthNet is real and used for pickup success confirmation.
- Arm and base actions are kept in the notebook so parameters can be tuned fast.

Files:

- `depth_camera.py`: reusable JetBot Camera + DepthNet wrapper.
- `can_pickup_depth_demo.ipynb`: main test workflow with editable movement,
  calibration, pickup, and confirmation logic.

Suggested flow:

1. Start camera and DepthNet.
2. Calibrate a baseline depth while the target can is visible.
3. Manually place the robot/can in the test pose.
4. Optionally run a hardcoded base movement snippet.
5. Run the arm pickup sequence.
6. Use depth readings to check whether the can area changed after pickup.
7. Reset arm in `finally`.

This is intentionally not a framework. Keep the logic simple until the real
control architecture is clearer.
