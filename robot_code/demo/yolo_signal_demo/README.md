# YOLO Signal Demo

Temporary onboard test for:

1. JetBot camera frame capture.
2. YOLO detection using the local `best.pt`.
3. Printed detection signal.
4. Small servo/gripper pulse to confirm the robot received the signal.

Run on the Jetson/JETANK:

```bash
cd /workspace/robot_code/demo/yolo_signal_demo
python3 run_yolo_signal_demo.py
```

Safer dry run, with no servo movement:

```bash
python3 run_yolo_signal_demo.py --no-arm
```

Useful options:

```bash
python3 run_yolo_signal_demo.py --seconds 30 --conf 0.35 --cooldown 4
```

Notes:

- Keep the robot in a safe position. The script only moves servo 4 slightly, but watch for cable or gripper interference.
- `best.pt` is copied into this folder so the demo can be transferred as one temporary folder.
- This does not use MiDaS/depth yet. It tests the camera -> YOLO -> signal -> servo-action loop.

