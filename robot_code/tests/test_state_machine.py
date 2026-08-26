import os
import unittest

import numpy as np

from demo_core import DemoStateMachine, MissionEvent, MissionState, RobotComponents, load_config


class FakeBase(object):
    def __init__(self):
        self.commands = []

    def pulse(self, direction, speed, seconds, label):
        self.commands.append((label, direction, float(speed), float(seconds)))

    def stop(self):
        self.commands.append(("stop",))


class FakeArm(object):
    def __init__(self):
        self.poses = []

    def pose(self, name, pose=None):
        self.poses.append((name, dict(pose or {})))
        return {1: 500}

    def wait_for_positions(self, targets, label):
        return True


class FakeDepth(object):
    def __init__(self):
        self.frame = np.zeros((240, 320, 3), dtype=np.uint8)
        self.started = False

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def read_frame(self):
        return self.frame

    def obstacle_detected_frame(self, frame):
        return False

    def observe_lens_center_frame(self, frame):
        return {"mean": 1.0, "min": 1.0, "max": 1.0}

    def depth_map_frame(self, frame):
        return np.full((240, 320), 2.0, dtype=np.float32)

    def grab_verified(self):
        return True


class FakeDetector(object):
    def __init__(self, kind):
        self.kind = kind

    def detect(self, frame):
        return {
            "kind": self.kind,
            "found": True,
            "confidence": 0.9,
            "bbox": [120.0, 40.0, 200.0, 180.0],
            "bbox_height_norm": 0.58,
            "center_x": 160.0,
            "center_y": 110.0,
            "error_x": 0.0,
        }

    def confidence_threshold(self, tracking=False):
        return 0.2


class StateMachineTest(unittest.TestCase):
    def test_delivery_asset_paths_resolve_from_config_directory(self):
        config = load_config()

        settings = config.section("detectors")["can"]
        self.assertTrue(config.resolve_path(settings["model_path"]).endswith(
            os.path.join("assets", "models", "detectnet_native_can", "can_ssd_mobilenet_v1.onnx")
        ))
        self.assertTrue(os.path.exists(config.resolve_path(settings["model_path"])))
        self.assertTrue(os.path.exists(config.resolve_path(settings["labels_path"])))
        self.assertTrue(config.get("runtime.dry_run.arm"))

    def test_command_overrides_take_precedence_over_runtime_config(self):
        config = load_config(overrides={
            "runtime": {"dry_run": {"arm": False}},
        })
        self.assertFalse(config.get("runtime.dry_run.arm"))

    def test_full_can_to_bin_mission_reaches_done_and_counts_pickup(self):
        pose_overrides = {}
        for name in ("safe_home", "arm_down", "grab", "carry", "release"):
            pose_overrides[name] = {"pause_seconds": 0.0}
        config = load_config(overrides={
            "runtime": {"loop_pause_seconds": 0.0},
            "navigation": {
                "can": {"approach": {"final_verify_frames": 2}},
                "bin": {"approach": {"final_verify_frames": 2}},
            },
            "arm": {
                "pickup_start_delay_seconds": 0.0,
                "push": {"speed": 0.0, "seconds": 0.0, "post_lock_seconds": 0.0},
                "poses": pose_overrides,
            },
        })
        base = FakeBase()
        arm = FakeArm()
        depth = FakeDepth()
        services = RobotComponents(base, arm, depth, FakeDetector("can"), FakeDetector("bin_tag"))
        runtime = DemoStateMachine(config, services=services)

        self.assertTrue(runtime.run(max_ticks=100))
        self.assertEqual(runtime.state, MissionState.DONE)
        self.assertEqual(runtime.context.completed_pickups, 1)
        self.assertFalse(runtime.context.grabbed)
        self.assertGreater(runtime.context.metrics["detector_calls"]["can"], 0)
        self.assertGreater(runtime.context.metrics["detector_calls"]["bin"], 0)
        self.assertIn("arm_down", [name for name, _ in arm.poses])
        self.assertIn("release", [name for name, _ in arm.poses])

    def test_intermediate_retries_then_exhausts_limit(self):
        config = load_config(overrides={"runtime": {"retry_limit": 1}})
        runtime = DemoStateMachine(
            config,
            services=RobotComponents(
                FakeBase(), FakeArm(), FakeDepth(), FakeDetector("can"), FakeDetector("bin_tag")
            ),
        )
        runtime.previous_state = MissionState.SEARCHING
        first = runtime._handle_intermediate_state()
        second = runtime._handle_intermediate_state()
        self.assertEqual(first.event, MissionEvent.RETRY)
        self.assertEqual(second.event, MissionEvent.RETRY_EXHAUSTED)


if __name__ == "__main__":
    unittest.main()
