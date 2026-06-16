import argparse
import time
from pathlib import Path

import cv2

try:
    from jetbot import Camera
except ImportError as exc:
    raise SystemExit("jetbot Camera is required. Run this on the Jetson/JETANK.") from exc

try:
    from ultralytics import YOLO
except ImportError as exc:
    raise SystemExit("ultralytics is required. Install it on the Jetson first.") from exc


HERE = Path(__file__).resolve().parent


def parse_args():
    parser = argparse.ArgumentParser(
        description="Camera -> YOLO -> printed signal -> small servo pulse demo."
    )
    parser.add_argument("--model", type=Path, default=HERE / "best.pt")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--conf", type=float, default=0.4)
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--width", type=int, default=416)
    parser.add_argument("--height", type=int, default=416)
    parser.add_argument("--cooldown", type=float, default=3.0)
    parser.add_argument("--max-actions", type=int, default=5)
    parser.add_argument("--no-arm", action="store_true")
    parser.add_argument("--save-last", action="store_true")
    return parser.parse_args()


class ArmSignal:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.ttl_servo = None
        if not enabled:
            print("[arm] disabled (--no-arm)")
            return

        try:
            from SCSCtrl import TTLServo
        except ImportError as exc:
            raise SystemExit("SCSCtrl is required for servo movement on JETANK.") from exc

        self.ttl_servo = TTLServo
        print("[arm] ready; using servo 4 small pulse")

    def pulse(self):
        if not self.enabled:
            print("[arm] signal pulse skipped")
            return

        # Servo 4 is the gripper in the JETANK tutorial. Keep movement tiny.
        self.ttl_servo.servoAngleCtrl(4, -15, 1, 150)
        time.sleep(0.25)
        self.ttl_servo.servoAngleCtrl(4, 0, 1, 150)
        time.sleep(0.25)
        print("[arm] signal pulse complete")


def best_detection(result):
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None

    confidences = boxes.conf.cpu().numpy()
    best_index = int(confidences.argmax())
    box = boxes.xyxy.cpu().numpy()[best_index]
    cls_id = int(boxes.cls.cpu().numpy()[best_index])
    conf = float(confidences[best_index])

    x1, y1, x2, y2 = map(int, box[:4])
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    area = max(0, x2 - x1) * max(0, y2 - y1)

    return {
        "name": result.names.get(cls_id, str(cls_id)),
        "conf": conf,
        "bbox": (x1, y1, x2, y2),
        "center": (center_x, center_y),
        "area": area,
    }


def main():
    args = parse_args()
    if not args.model.is_file():
        raise SystemExit(f"Model not found: {args.model}")

    print(f"[demo] loading model: {args.model}")
    model = YOLO(str(args.model))
    arm_signal = ArmSignal(enabled=not args.no_arm)
    camera = Camera.instance(width=args.width, height=args.height)

    print(
        f"[demo] running for {args.seconds:.1f}s, conf={args.conf}, "
        f"imgsz={args.imgsz}, frame={args.width}x{args.height}"
    )

    start_time = time.time()
    last_action_time = 0.0
    action_count = 0
    frame_count = 0
    last_frame = None

    try:
        while time.time() - start_time < args.seconds:
            frame = camera.value
            if frame is None:
                print("[camera] no frame yet")
                time.sleep(0.1)
                continue

            frame_count += 1
            last_frame = frame.copy()
            result = model.predict(
                frame,
                conf=args.conf,
                imgsz=args.imgsz,
                verbose=False,
            )[0]

            detection = best_detection(result)
            if detection is None:
                print(f"[frame {frame_count}] no detection")
                time.sleep(0.15)
                continue

            print(
                f"[frame {frame_count}] detected {detection['name']} "
                f"conf={detection['conf']:.2f} center={detection['center']} "
                f"bbox={detection['bbox']} area={detection['area']}"
            )

            now = time.time()
            if (
                now - last_action_time >= args.cooldown
                and action_count < args.max_actions
            ):
                action_count += 1
                last_action_time = now
                print(f"[signal] trigger #{action_count}")
                arm_signal.pulse()

            time.sleep(0.15)

    finally:
        camera.stop()
        print("[camera] stopped")

    if args.save_last and last_frame is not None:
        output_path = HERE / "last_frame.jpg"
        cv2.imwrite(str(output_path), last_frame)
        print(f"[demo] saved {output_path}")

    print(f"[demo] done; frames={frame_count}, actions={action_count}")


if __name__ == "__main__":
    main()

