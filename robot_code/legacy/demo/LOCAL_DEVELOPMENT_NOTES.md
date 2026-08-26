# Local Development Notes

This file is a local-only working note for the JETANK bring-up and vision/control integration session. It is intentionally ignored by Git and is not meant to be formal project documentation. Its purpose is to preserve context from hands-on testing, ambiguous hardware behavior, useful commands, and next-step ideas so future development does not have to rediscover them.

## Current Scope

The current work is focused on early robot bring-up and integration testing:

- Connect to the JETANK / Jetson Nano.
- Verify Wi-Fi + Jupyter access.
- Test basic chassis motion.
- Test camera access.
- Test servo / arm control.
- Verify that the onboard `jetson-inference` DepthNet environment can run from Python.
- Build simple demo loops where camera/model output triggers visible servo movement.

This is not yet the final autonomous pickup workflow. The current demos are signal/flow validation.

## Connection Bring-Up

- PuTTY serial connection uses baud rate `115200`.
- Windows may briefly show `USB Serial Device (COMx)` when the Jetson enumerates over Micro-USB.
- If COM appears only under "Show hidden devices", it is a stale historical device, not an active connection.
- The small switch state where the fan spins faster appears to be the real vehicle power-on state.
- Micro-USB can partially power the Jetson and make the fan spin slowly, but this is not reliable enough for normal operation.
- Once Wi-Fi is configured with `nmcli`, Jupyter can be accessed from Windows at:

```text
http://<jetson-ip>:8888
```

- Default Jupyter password worked as `jetbot`.
- After Wi-Fi is stable, USB/PuTTY is mostly a recovery path for IP lookup, Wi-Fi repair, or Jupyter failure.

Useful Jetson commands:

```bash
nmcli device wifi list
sudo nmcli device wifi connect "SSID" password "PASSWORD"
ip addr show wlan0
ifconfig wlan0
```

## Hardware Control Findings

Basic chassis control works from Python:

```python
from jetbot.robot import Robot
import time

robot = Robot()
try:
    robot.forward(0.15)
    time.sleep(0.2)
finally:
    robot.stop()
```

Important behavior:

- `robot.forward(speed)` sets a continuous motor speed.
- It is not a "step once" command.
- To make separate motion segments, call `stop()` between them.
- Always wrap motion tests in `try/finally` with `robot.stop()`.

Servo control uses the JETANK tutorial API:

```python
from SCSCtrl import TTLServo
TTLServo.servoAngleCtrl(servoID, angleInput, direction, speed)
```

Known servo IDs from tutorial:

- `1`: pan / horizontal camera-arm direction
- `2`: arm root pitch
- `3`: arm middle joint pitch
- `4`: gripper
- `5`: camera tilt

For early signal demos, we used small reversible motions:

- Pan: servo `1`, `+angle -> -angle -> 0`
- Tilt: servo `5`, `+angle -> -angle -> 0`
- Earlier gripper pulse: servo `4`

## Camera Findings

JetBot Camera works:

```python
from jetbot import Camera
camera = Camera.instance(width=320, height=240)
frame = camera.value
```

`frame` is a NumPy array like:

```text
(240, 320, 3) uint8
```

The official `jetson_utils.videoSource()` path did not work with this robot camera:

- `videoSource("csi://0")` created a source but returned no frames.
- `/dev/video0` exists, but official `video-viewer` / `depthnet` commands also failed to create a usable source.
- Therefore current working route is:

```text
JetBot Camera -> NumPy frame -> cv2 BGR/RGB conversion -> jetson_utils.cudaFromNumpy -> model
```

## Model / Inference Findings

The downloaded `robot_code/demo/new_model/` is a local copy of the vehicle's `/workspace/jetson-inference` directory. It is very large and ignored by Git.

Depth model currently used:

```text
/workspace/jetson-inference/data/networks/MonoDepth-FCN-Mobilenet/monodepth_fcn_mobilenet.onnx
/workspace/jetson-inference/data/networks/MonoDepth-FCN-Mobilenet/monodepth_fcn_mobilenet.onnx.1.1.7103.GPU.FP16.engine
/workspace/jetson-inference/data/networks/MonoDepth-FCN-Mobilenet/monodepth_fcn_mobilenet.onnx.sha256sum
```

The `.engine` is the TensorRT-optimized cache. It is loaded successfully.

Working DepthNet path:

```python
from jetbot import Camera
from jetson_inference import depthNet
from jetson_utils import cudaFromNumpy, cudaToNumpy, cudaDeviceSynchronize

camera = Camera.instance(width=320, height=240)
net = depthNet("fcn-mobilenet")
depth_field = net.GetDepthField()
depth_numpy = cudaToNumpy(depth_field)

frame = camera.value
rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
cuda_img = cudaFromNumpy(rgb)
net.Process(cuda_img)
cudaDeviceSynchronize()
```

`depth_numpy` gives relative depth values, not calibrated metric distance.

Another model seen in `jetson-inference`:

```text
/workspace/jetson-inference/data/networks/SSD-Mobilenet-v2/
```

Important files:

```text
ssd_mobilenet_v2_coco.uff
ssd_mobilenet_v2_coco.uff.1.1.7103.GPU.FP16.engine
ssd_coco_labels.txt
```

This is a COCO detectNet model and may detect `bottle`. It should use the same JetBot Camera + `cudaFromNumpy` input path, because `videoSource` is not working on this robot.

## Created Demo Files

Current useful files:

```text
robot_code/demo/depthnet_arm_signal_demo.py
robot_code/demo/depthnet_servo_decision_demo.ipynb
```

Purpose:

- Use JetBot Camera.
- Run DepthNet.
- Print center depth statistics.
- Trigger tiny servo actions using `servoAngleCtrl()`.

Current decision demo logic:

```text
if center_depth_mean < 1.6:
    servo 1 pan left/right
else:
    servo 5 tilt up/down
```

This is just a visible signal mapping, not a real grasping policy.

Ignored local directories/files:

```text
robot_code/demo/new_model/
robot_code/demo/yolo_signal_demo/best.pt
robot_code/demo/LOCAL_DEVELOPMENT_NOTES.md
```

## Project Boundary and Cleanup Notes

- Do not modify `/workspace/jetson-inference` as project code unless deliberately patching third-party inference support.
- Do not develop inside `/workspace/jetbot` unless changing the JetBot library itself.
- Project code should live under `/workspace/robot_code` on the vehicle and under `robot_code/` in this repository.
- The local `new_model` folder is a copied environment snapshot and should not be committed.
- The earlier YOLO `.pt` path was not useful for this Jetson integration path. It is separate from `jetson-inference` detectNet.

## Useful Design Discussion

Bottle pickup can logically combine:

- `detectNet`: find bottle bbox, confidence, center position.
- `depthNet`: estimate relative depth at the bottle bbox center or bbox central region.

Potential flow:

```text
camera
  -> detect bottle
  -> choose best/most-centered bottle
  -> run depth
  -> sample depth over bbox center region
  -> decide left/right/center and far/near/pickup range
  -> trigger chassis or arm state
```

Important idea for later:

- Create a small intermediate API / data object that collects:
  - bbox
  - class name
  - confidence
  - image center error
  - bbox area/height
  - sampled depth stats
  - derived state such as `left`, `right`, `center`, `far`, `pickup_range`

This should improve readability with negligible performance cost, because it is just packaging already-computed values.

## Known Oddities

- Headless runs often print:

```text
nvbuf_utils: Could not get EGL display connection
```

This did not prevent successful camera/model operation in the tested path.

- Some Jetson Argus/GStreamer cleanup can produce warnings or even segfault after work is done. The successful signal is that the frame was processed and printed before cleanup.
- A directory/file named like a Wi-Fi command appeared in `/workspace`; it likely came from accidentally creating a file from a pasted command and contains a password. Do not commit or share it.

## Next Useful Step

Build a bottle detection demo using `detectNet("ssd-mobilenet-v2")` with the proven input path:

```text
JetBot Camera -> NumPy -> cudaFromNumpy -> detectNet.Detect()
```

Then combine bottle bbox with DepthNet bbox-region depth sampling and use servo signal motions to indicate:

- bottle left/right
- bottle centered
- bottle near/far
- pickup trigger condition

