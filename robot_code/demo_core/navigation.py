from __future__ import print_function

import time

from .fsm_types import MissionEvent, TargetType


class StepOutcome(object):
    def __init__(self, event=None, observation=None, reason=None):
        self.event = event
        self.observation = observation
        self.reason = reason


class TargetNavigator(object):
    """Incremental navigation shared by can and bin targets."""

    def __init__(self, config, context, base, depth, can_detector, bin_detector):
        self.config = config
        self.context = context
        self.base = base
        self.depth = depth
        self.can_detector = can_detector
        self.bin_detector = bin_detector

    def detector(self, target_type):
        return self.can_detector if target_type == TargetType.CAN else self.bin_detector

    def detect(self, target_type, frame=None):
        if frame is None:
            frame = self.depth.read_frame()
        if frame is None:
            return {"found": False, "reason": "no_frame"}
        start = time.time()
        observation = self.detector(target_type).detect(frame)
        self.context.record_detection(target_type, time.time() - start)
        return observation

    def _accepted(self, target_type, observation, tracking=False):
        if not observation or not observation.get("found"):
            return False
        if target_type != TargetType.CAN:
            return True
        threshold = self.can_detector.confidence_threshold(tracking)
        return float(observation.get("confidence", 0.0)) >= threshold

    def search_step(self, target_type):
        settings = self.config.target_navigation(target_type)["search"]
        if self.context.elapsed() >= float(settings["timeout_seconds"]):
            self.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="search timeout")
        observation = self.detect(target_type)
        if self._accepted(target_type, observation, tracking=False):
            self.base.stop()
            return StepOutcome(MissionEvent.TARGET_FOUND, observation)
        self.base.pulse(
            settings["direction"],
            float(settings["speed"]),
            float(settings["pulse_seconds"]),
            "search_{}".format(target_type.value),
        )
        self.context.increment_step()
        return StepOutcome(observation=observation)

    def align_step(self, target_type, settings=None):
        settings = settings or self.config.target_navigation(target_type)["align"]
        if self.context.increment_step() > int(settings["max_steps"]):
            self.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="alignment max steps")
        observation = self.detect(target_type)
        if not self._accepted(target_type, observation, tracking=True):
            lost = int(self.context.state_data.get("lost_frames", 0)) + 1
            self.context.state_data["lost_frames"] = lost
            if lost >= int(settings.get("lost_frame_limit", 5)):
                self.base.stop()
                return StepOutcome(MissionEvent.TARGET_MISSING, observation, "alignment target lost")
            time.sleep(float(self.config.get("camera.observation_pause_seconds", 0.1)))
            return StepOutcome(observation=observation)
        self.context.state_data["lost_frames"] = 0
        error_x = float(observation["error_x"])
        if abs(error_x) <= float(settings["tolerance_norm"]):
            self.base.stop()
            return StepOutcome(MissionEvent.TARGET_ALIGNED, observation)
        direction = "right" if error_x > 0 else "left"
        self.base.pulse(
            direction,
            float(settings["speed"]),
            float(settings["pulse_seconds"]),
            "align_{}".format(target_type.value),
        )
        return StepOutcome(observation=observation)

    def _depth_stop_value(self, frame, stop):
        stats = self.depth.observe_lens_center_frame(frame)
        if not stats:
            return None
        raw = float(stats["mean"])
        self.context.distance_to_target = raw
        return raw * float(stop.get("depth_scale", 1.0))

    def _stop_value(self, target_type, frame, observation, stop):
        mode = stop["mode"]
        if mode == "bbox_height":
            return observation.get("bbox_height_norm") if observation else None
        if mode == "depth":
            return self._depth_stop_value(frame, stop)
        raise ValueError("unsupported approach stop mode: {}".format(mode))

    def _stop_reached(self, value, stop):
        if value is None:
            return False
        threshold = float(stop["threshold"])
        if stop["mode"] == "depth":
            return float(value) <= threshold
        return float(value) >= threshold

    def approach_step(self, target_type):
        settings = self.config.target_navigation(target_type)["approach"]
        if self.context.elapsed() >= float(settings["timeout_seconds"]):
            self.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="approach timeout")
        if self.context.increment_step() > int(settings.get("max_pulses", 0) or 1000000):
            self.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="approach max pulses")
        frame = self.depth.read_frame()
        if frame is None:
            self.base.stop()
            return StepOutcome(MissionEvent.FAIL, reason="camera frame unavailable")
        if self.config.get("avoidance.strategy", "disabled") != "disabled" and self.depth.obstacle_detected_frame(frame):
            self.base.stop()
            self.context.obstacle_found = True
            return StepOutcome(MissionEvent.OBSTACLE_FOUND, reason="depth obstacle")
        observation = self.detect(target_type, frame)
        if not self._accepted(target_type, observation, tracking=True):
            lost = int(self.context.state_data.get("lost_frames", 0)) + 1
            self.context.state_data["lost_frames"] = lost
            self.base.stop()
            if lost >= int(settings.get("lost_frame_limit", 4)):
                return StepOutcome(MissionEvent.TARGET_MISSING, observation, "approach target lost")
            time.sleep(float(self.config.get("camera.observation_pause_seconds", 0.1)))
            return StepOutcome(observation=observation)
        self.context.state_data["lost_frames"] = 0
        stop = settings["stop"]
        stop_value = self._stop_value(target_type, frame, observation, stop)
        print(
            "[approach] target={} mode={} value={} threshold={} error_x={:.3f}".format(
                target_type.value,
                stop["mode"],
                "{:.3f}".format(float(stop_value)) if stop_value is not None else "n/a",
                stop["threshold"],
                float(observation["error_x"]),
            )
        )
        if self.context.state_data["steps"] > int(settings.get("min_pulses", 0)) and self._stop_reached(stop_value, stop):
            self.base.stop()
            return StepOutcome(MissionEvent.TARGET_REACHED, observation)
        error_x = float(observation["error_x"])
        if abs(error_x) > float(settings.get("steering_tolerance_norm", 1.0)):
            direction = "right" if error_x > 0 else "left"
            self.base.pulse(
                direction,
                float(settings.get("steering_speed", settings["speed"])),
                float(settings.get("steering_pulse_seconds", settings["pulse_seconds"])),
                "approach_{}_steer".format(target_type.value),
            )
        else:
            self.base.pulse(
                "forward",
                float(settings["speed"]),
                float(settings["pulse_seconds"]),
                "approach_{}".format(target_type.value),
            )
        return StepOutcome(observation=observation)

    def final_verify_step(self, target_type):
        navigation = self.config.target_navigation(target_type)
        settings = navigation.get("near_align", navigation["align"])
        if not bool(settings.get("enabled", True)):
            return StepOutcome(MissionEvent.TARGET_STABLE, self.context.last_observation)
        outcome = self.align_step(target_type, settings=settings)
        if outcome.event == MissionEvent.TARGET_ALIGNED:
            stable = int(self.context.state_data.get("stable_frames", 0)) + 1
            self.context.state_data["stable_frames"] = stable
            required = int(navigation["approach"].get("final_verify_frames", 2))
            if stable >= required:
                return StepOutcome(MissionEvent.TARGET_STABLE, outcome.observation)
            return StepOutcome(observation=outcome.observation)
        self.context.state_data["stable_frames"] = 0
        return outcome
