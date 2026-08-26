import json
import time
from pathlib import Path

from depth_camera import DepthCamera


BASE_DIR = Path(__file__).resolve().parent
GRAB_PARAM_PATH = BASE_DIR / "arm_grab_tuning_params.json"
MOTION_PARAM_PATH = BASE_DIR / "motion_pickup_demo_params.json"


DEFAULT_GRAB_PARAMS = {
    "safe_s1": 0,
    "safe_s2": 0,
    "safe_s3": 0,
    "safe_s4": 0,
    "safe_s5": 0,
    "ready_s1": 0,
    "ready_s2": 18,
    "ready_s3": -12,
    "ready_s4": 0,
    "ready_s5": 0,
    "pick_s1": 0,
    "pick_s2": 5,
    "pick_s3": 18,
    "pick_s4": -55,
    "pick_s5": 0,
    "lift_state_s1": 0,
    "lift_state_s2": 0,
    "lift_state_s3": 28,
    "lift_state_s4": -55,
    "lift_state_s5": 0,
    "pre_s2": 18,
    "pre_s3": -12,
    "reach_s2": 26,
    "reach_s3": -22,
    "lift_s2": 5,
    "lift_s3": 18,
    "gripper_open": 0,
    "gripper_close_1": -45,
    "gripper_close_2": -55,
    "arm_speed": 80,
    "gripper_speed": 120,
    "settle_seconds": 0.35,
}


DEFAULT_MOTION_PARAMS = {
    "dry_run_base": True,
    "dry_run_arm": True,
    "skip_depth_check": False,
    "depth_width": 320,
    "depth_height": 240,
    "depth_network": "fcn-mobilenet",
    "depth_region_x1": 0.40,
    "depth_region_y1": 0.40,
    "depth_region_x2": 0.60,
    "depth_region_y2": 0.60,
    "pickup_success_depth_threshold": 1.60,
    "scan_turn_direction": "left",
    "scan_turn_speed": 0.12,
    "scan_turn_seconds": 1.0,
    "scan_stop_angle_deg": 90,
    "approach_direction": "forward",
    "approach_speed": 0.15,
    "approach_seconds": 1.2,
    "approach_distance_m": 0.5,
    "drop_turn_direction": "right",
    "drop_turn_speed": 0.12,
    "drop_turn_seconds": 1.0,
    "drop_stop_angle_deg": 90,
    "drop_drive_direction": "forward",
    "drop_drive_speed": 0.15,
    "drop_drive_seconds": 1.2,
    "drop_drive_distance_m": 0.5,
    "pose_after_grab_s1": 0,
    "pose_after_grab_s2": 8,
    "pose_after_grab_s3": 10,
    "pose_after_grab_s4": -55,
    "pose_after_grab_s5": 0,
    "drop_pose_s1": 0,
    "drop_pose_s2": 18,
    "drop_pose_s3": -10,
    "drop_pose_s4": -55,
    "drop_pose_s5": 0,
    "release_grip": 0,
    "arm_speed": 80,
    "gripper_speed": 120,
    "settle_seconds": 0.35,
}


def load_params(path, defaults):
    params = dict(defaults)
    if path.exists():
        with path.open("r") as f:
            params.update(json.load(f))
        print("[params] loaded {}".format(path))
    else:
        print("[params] using defaults; missing {}".format(path))
    return params


class MotionPickupDemo(object):
    def __init__(self, grab_params, motion_params):
        self.grab_params = grab_params
        self.motion_params = motion_params
        self.depth = None
        self.robot = None
        self.ttl_servo = None

    def depth_region(self):
        p = self.motion_params
        return (
            p["depth_region_x1"],
            p["depth_region_y1"],
            p["depth_region_x2"],
            p["depth_region_y2"],
        )

    def start_depth_and_verify(self, frame_count=2):
        if self.depth is None:
            print("[depth] starting network and camera")
            self.depth = DepthCamera(
                width=int(self.motion_params["depth_width"]),
                height=int(self.motion_params["depth_height"]),
                network=str(self.motion_params["depth_network"]),
            )
            self.depth.start(warmup_frames=1)

        print("[depth] verifying {} frames".format(frame_count))
        verified = 0
        attempts = 0
        while verified < frame_count and attempts < frame_count * 10:
            attempts += 1
            stats = self.depth.observe(region=self.depth_region())
            if stats is None:
                print("[depth] verify attempt={} no frame".format(attempts))
                time.sleep(0.2)
                continue
            verified += 1
            print(
                "[depth] verify frame={} mean={:.3f} min={:.3f} max={:.3f}".format(
                    verified, stats["mean"], stats["min"], stats["max"]
                )
            )
            time.sleep(0.2)

        if verified < frame_count:
            raise RuntimeError("depth camera did not return enough frames")

    def ensure_robot(self):
        if self.motion_params["dry_run_base"]:
            return None
        if self.robot is None:
            from jetbot import Robot

            self.robot = Robot()
            print("[base] Robot connected")
        return self.robot

    def ensure_servos(self):
        if self.motion_params["dry_run_arm"]:
            return None
        if self.ttl_servo is None:
            from SCSCtrl import TTLServo

            self.ttl_servo = TTLServo
            print("[arm] TTLServo connected")
        return self.ttl_servo

    def base_stop(self):
        if self.robot is not None:
            self.robot.stop()
        print("[base] stop")

    def base_drive(self, direction, speed, seconds, label):
        bot = self.ensure_robot()
        print(
            "[base] {} direction={} speed={} seconds={}".format(
                label, direction, speed, seconds
            )
        )
        if bot is None:
            time.sleep(float(seconds))
            print("[base] {} done".format(label))
            return

        try:
            if direction == "forward":
                bot.forward(float(speed))
            elif direction == "backward":
                bot.backward(float(speed))
            elif direction == "left":
                bot.left(float(speed))
            elif direction == "right":
                bot.right(float(speed))
            else:
                raise ValueError("unknown base direction: {}".format(direction))
            time.sleep(float(seconds))
        finally:
            bot.stop()
        print("[base] {} done".format(label))

    def move_servo(self, servo_id, angle, speed, label=""):
        servos = self.ensure_servos()
        print(
            "[arm] servo={} angle={} speed={} {}".format(
                servo_id, angle, speed, label
            )
        )
        if servos is not None:
            servos.servoAngleCtrl(int(servo_id), int(angle), 1, int(speed))
        time.sleep(float(self.motion_params["settle_seconds"]))

    def apply_pose(self, name, pose, speed):
        print("[arm] pose: {}".format(name))
        for servo_id, angle in pose:
            self.move_servo(servo_id, angle, speed, name)

    def grab_value(self, name):
        return self.grab_params[name]

    def safe_home(self):
        pose = [
            (1, self.grab_value("safe_s1")),
            (2, self.grab_value("safe_s2")),
            (3, self.grab_value("safe_s3")),
            (4, self.grab_value("safe_s4")),
            (5, self.grab_value("safe_s5")),
        ]
        self.apply_pose("safe_home", pose, self.grab_value("arm_speed"))

    def ready_state(self):
        pose = [
            (1, self.grab_value("ready_s1")),
            (2, self.grab_value("ready_s2")),
            (3, self.grab_value("ready_s3")),
            (4, self.grab_value("ready_s4")),
            (5, self.grab_value("ready_s5")),
        ]
        self.apply_pose("ready_state", pose, self.grab_value("arm_speed"))

    def open_gripper(self):
        self.move_servo(
            4,
            self.grab_value("gripper_open"),
            self.grab_value("gripper_speed"),
            "open_gripper",
        )

    def close_gripper(self):
        self.move_servo(
            4,
            self.grab_value("gripper_close_1"),
            self.grab_value("gripper_speed"),
            "close_1",
        )
        time.sleep(0.5)
        self.move_servo(
            4,
            self.grab_value("gripper_close_2"),
            self.grab_value("gripper_speed"),
            "close_2",
        )

    def pick_state(self):
        pose = [
            (1, self.grab_value("pick_s1")),
            (2, self.grab_value("pick_s2")),
            (3, self.grab_value("pick_s3")),
            (4, self.grab_value("pick_s4")),
            (5, self.grab_value("pick_s5")),
        ]
        self.apply_pose("pick_state", pose, self.grab_value("arm_speed"))

    def lift_state(self):
        pose = [
            (1, self.grab_value("lift_state_s1")),
            (2, self.grab_value("lift_state_s2")),
            (3, self.grab_value("lift_state_s3")),
            (4, self.grab_value("lift_state_s4")),
            (5, self.grab_value("lift_state_s5")),
        ]
        self.apply_pose("lift_state", pose, self.grab_value("arm_speed"))

    def grab_sequence_without_final_home(self):
        print("[flow] grab sequence using {}".format(GRAB_PARAM_PATH))
        self.ready_state()
        self.open_gripper()
        self.apply_pose(
            "pre_grasp",
            [(2, self.grab_value("pre_s2")), (3, self.grab_value("pre_s3"))],
            self.grab_value("arm_speed"),
        )
        self.apply_pose(
            "reach",
            [(2, self.grab_value("reach_s2")), (3, self.grab_value("reach_s3"))],
            self.grab_value("arm_speed"),
        )
        self.close_gripper()
        self.pick_state()
        self.lift_state()

    def after_grab_pose(self):
        p = self.motion_params
        pose = [
            (1, p["pose_after_grab_s1"]),
            (2, p["pose_after_grab_s2"]),
            (3, p["pose_after_grab_s3"]),
            (4, p["pose_after_grab_s4"]),
            (5, p["pose_after_grab_s5"]),
        ]
        self.apply_pose("after_grab_pose", pose, p["arm_speed"])

    def drop_release_sequence(self):
        p = self.motion_params
        pose = [
            (1, p["drop_pose_s1"]),
            (2, p["drop_pose_s2"]),
            (3, p["drop_pose_s3"]),
            (4, p["drop_pose_s4"]),
            (5, p["drop_pose_s5"]),
        ]
        self.apply_pose("drop_pose", pose, p["arm_speed"])
        self.move_servo(4, p["release_grip"], p["gripper_speed"], "release_grip")

    def observe_pickup_depth(self, label):
        if self.motion_params.get("skip_depth_check"):
            print(
                "[depth] skip_depth_check=True; assume can was found and pickup succeeded"
            )
            return True
        if self.depth is None:
            raise RuntimeError("depth camera is not started")
        stats = self.depth.observe(region=self.depth_region())
        if stats is None:
            print("[depth] {} no frame; grabbed=False".format(label))
            return False

        threshold = float(self.motion_params["pickup_success_depth_threshold"])
        success = stats["mean"] < threshold
        print(
            "[depth] {} mean={:.3f} min={:.3f} max={:.3f} threshold={:.3f} grabbed={}".format(
                label, stats["mean"], stats["min"], stats["max"], threshold, success
            )
        )
        return success

    def print_points(self):
        p = self.motion_params
        print(
            "[points] A start=({:.2f}, {:.2f}) B can=({:.2f}, {:.2f}) C drop=({:.2f}, {:.2f})".format(
                p.get("start_x", 0.0),
                p.get("start_y", 0.0),
                p.get("can_x", 0.0),
                p.get("can_y", 0.0),
                p.get("drop_x", 0.0),
                p.get("drop_y", 0.0),
            )
        )
        print(
            "[move] scan stop angle={} deg, approach distance={} m".format(
                p["scan_stop_angle_deg"], p["approach_distance_m"]
            )
        )
        print(
            "[move] drop turn stop angle={} deg, drop drive distance={} m".format(
                p["drop_stop_angle_deg"], p["drop_drive_distance_m"]
            )
        )

    def run(self):
        p = self.motion_params
        print("[flow] motion pickup demo start")
        print(
            "[flow] dry_run_base={} dry_run_arm={} skip_depth_check={}".format(
                p["dry_run_base"],
                p["dry_run_arm"],
                p.get("skip_depth_check"),
            )
        )
        self.print_points()

        try:
            if p.get("skip_depth_check"):
                print(
                    "[depth] skip_depth_check=True; skip DepthNet startup and assume target is available"
                )
            else:
                self.start_depth_and_verify(frame_count=2)
            self.safe_home()
            self.base_stop()

            print("[flow] rotate in place to simulate target scan")
            self.base_drive(
                p["scan_turn_direction"],
                p["scan_turn_speed"],
                p["scan_turn_seconds"],
                "scan_turn_to_angle_{}".format(p["scan_stop_angle_deg"]),
            )

            print("[flow] hardcoded move from A to B")
            self.base_drive(
                p["approach_direction"],
                p["approach_speed"],
                p["approach_seconds"],
                "approach_can_distance_{}m".format(p["approach_distance_m"]),
            )

            print("[flow] ready state near can")
            self.ready_state()

            print("[flow] grab can")
            self.grab_sequence_without_final_home()

            success = self.observe_pickup_depth("after_grab_lift")
            if not success:
                print("[flow] pickup failed by depth threshold; return to safe_home")
                self.safe_home()
                return False

            print("[flow] pickup success; move to carrying posture")
            self.after_grab_pose()

            print("[flow] rotate in place to simulate drop-point scan")
            self.base_drive(
                p["drop_turn_direction"],
                p["drop_turn_speed"],
                p["drop_turn_seconds"],
                "drop_turn_to_angle_{}".format(p["drop_stop_angle_deg"]),
            )

            print("[flow] hardcoded move from B to C")
            self.base_drive(
                p["drop_drive_direction"],
                p["drop_drive_speed"],
                p["drop_drive_seconds"],
                "drive_to_drop_distance_{}m".format(p["drop_drive_distance_m"]),
            )

            print("[flow] release at drop point")
            self.drop_release_sequence()
            print("[flow] demo done")
            return True
        finally:
            print("[flow] final cleanup: stop base and safe_home")
            self.base_stop()
            self.safe_home()
            if self.depth is not None:
                self.depth.stop()


def main():
    grab_params = load_params(GRAB_PARAM_PATH, DEFAULT_GRAB_PARAMS)
    motion_params = load_params(MOTION_PARAM_PATH, DEFAULT_MOTION_PARAMS)
    demo = MotionPickupDemo(grab_params, motion_params)
    success = demo.run()
    print("[result] success={}".format(success))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
