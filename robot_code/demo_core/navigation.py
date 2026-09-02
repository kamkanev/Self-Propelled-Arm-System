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

    def __init__(self, config, context, base, depth, can_detector, bin_detector, frame_observer=None):
        self.config = config
        self.context = context
        self.base = base
        self.depth = depth
        self.can_detector = can_detector
        self.bin_detector = bin_detector
        self.frame_observer = frame_observer

    def detector(self, target_type):
        return self.can_detector if target_type == TargetType.CAN else self.bin_detector

    def detect(self, target_type, frame=None):
        if frame is None:
            frame = self.depth.read_frame()
        if frame is None:
            return {"found": False, "reason": "no_frame"}
        if self.frame_observer is not None:
            self.frame_observer(target_type, frame)
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

    def _start_visual_turn(self, direction, configured_speed, label):
        speed = float(self.config.get("base_motion_speed_profiles.turn.slow", configured_speed))
        if self.base.start_motion(direction, speed, label):
            print("[visual-turn] label={} direction={} speed={}".format(label, direction, speed))

    def search_step(self, target_type):
        settings = self.config.target_navigation(target_type)["search"]
        label = "search_{}".format(target_type.value)
        if self.context.elapsed() >= float(settings["timeout_seconds"]):
            self.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="search timeout")
        observation = self.detect(target_type)
        if self._accepted(target_type, observation, tracking=False):
            self.base.stop()
            return StepOutcome(MissionEvent.TARGET_FOUND, observation)
        self.base.start_motion(settings["direction"], float(settings["speed"]), label)
        self.context.increment_step()
        return StepOutcome(observation=observation)

    def align_step(self, target_type, settings=None):
        settings = settings or self.config.target_navigation(target_type)["align"]
        if self.context.increment_step() > int(settings["max_steps"]):
            self.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="alignment max steps")
        observation = self.detect(target_type)
        if not self._accepted(target_type, observation, tracking=True):
            self.base.stop()
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
        self._start_visual_turn(direction, settings["speed"], "align_{}".format(target_type.value))
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
        experiment = settings.get("experimental_continuous", {})
        if target_type == TargetType.CAN and bool(experiment.get("enabled", False)):
            return self._experimental_continuous_can_approach(settings, experiment)
        forward_label = "approach_{}_forward".format(target_type.value)
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
            self._start_visual_turn(
                direction,
                settings.get("steering_speed", settings["speed"]),
                "approach_{}_steer".format(target_type.value),
            )
        else:
            self.base.start_motion("forward", float(settings["speed"]), forward_label)
        return StepOutcome(observation=observation)

    def _experimental_continuous_can_approach(self, settings, experiment):
        label = "exp1_can_approach"
        duration = float(experiment["duration_seconds"])
        speed = float(experiment["speed"])
        active_seconds = float(self.context.state_data.get("exp1_active_seconds", 0.0))
        now = time.time()
        moving = self.base.motion_active(label)

        if moving:
            last_tick = float(self.context.state_data.get("exp1_last_tick", now))
            active_seconds += max(0.0, now - last_tick)
            self.context.state_data["exp1_active_seconds"] = active_seconds
        self.context.state_data["exp1_last_tick"] = now

        frame = self.depth.read_frame()
        if frame is None:
            self.base.stop()
            return StepOutcome(MissionEvent.FAIL, reason="exp1 camera frame unavailable")
        if self.config.get("avoidance.strategy", "disabled") != "disabled" and self.depth.obstacle_detected_frame(frame):
            self.base.stop()
            self.context.obstacle_found = True
            return StepOutcome(MissionEvent.OBSTACLE_FOUND, reason="exp1 depth obstacle")
        observation = self.detect(TargetType.CAN, frame)
        self.context.increment_step()

        if not self._accepted(TargetType.CAN, observation, tracking=True):
            if moving:
                active_seconds += max(0.0, time.time() - now)
                self.context.state_data["exp1_active_seconds"] = active_seconds
            self.base.stop()
            lost = int(self.context.state_data.get("lost_frames", 0)) + 1
            self.context.state_data["lost_frames"] = lost
            if lost >= int(settings.get("lost_frame_limit", 4)):
                return StepOutcome(MissionEvent.TARGET_MISSING, observation, "exp1 approach target lost")
            return StepOutcome(observation=observation)
        self.context.state_data["lost_frames"] = 0

        if active_seconds >= duration:
            self.base.stop()
            print("[exp1] continuous can approach complete active_seconds={:.3f}".format(active_seconds))
            return StepOutcome(MissionEvent.TARGET_REACHED, observation)

        error_x = float(observation["error_x"])
        if abs(error_x) > float(settings.get("steering_tolerance_norm", 1.0)):
            if moving:
                active_seconds += max(0.0, time.time() - now)
                self.context.state_data["exp1_active_seconds"] = active_seconds
            direction = "right" if error_x > 0 else "left"
            self._start_visual_turn(
                direction,
                settings.get("steering_speed", settings["speed"]),
                "exp1_can_steer",
            )
            print("[exp1] visual steering direction={} error_x={:.3f}".format(direction, error_x))
            return StepOutcome(observation=observation)

        if not moving:
            self.base.start_motion("forward", speed, label)
            self.context.state_data["exp1_last_tick"] = time.time()
            print("[exp1] continuous displacement speed={} duration_seconds={} remaining={:.3f}".format(
                speed, duration, max(0.0, duration - active_seconds)
            ))
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


class BinSideDockingNavigator(object):
    """Incremental right-facing AprilTag calibration used by exp2."""

    def __init__(self, config, context, base, depth, detector):
        self.config = config
        self.context = context
        self.base = base
        self.depth = depth
        self.detector = detector

    def _turn_for_signed_error(self, error, positive_direction, speed, label):
        direction = positive_direction if error > 0.0 else self._opposite(positive_direction)
        slow_speed = float(self.config.get("base_motion_speed_profiles.turn.slow", speed))
        if self.base.start_motion(direction, slow_speed, label):
            print("[visual-turn] label={} direction={} speed={}".format(label, direction, slow_speed))
        return direction

    @staticmethod
    def _opposite(direction):
        return {
            "left": "right",
            "right": "left",
            "forward": "backward",
            "backward": "forward",
        }[direction]

    def _standoff_shift(self, toward_bin, settings):
        turn_direction = settings["standoff_toward_turn_direction"]
        if not toward_bin:
            turn_direction = self._opposite(turn_direction)
        return_direction = self._opposite(turn_direction)
        self.base.pulse(turn_direction, settings["standoff_turn_speed"], settings["standoff_turn_seconds"], "exp2_standoff_turn")
        self.base.pulse("forward", settings["standoff_drive_speed"], settings["standoff_drive_seconds"], "exp2_standoff_drive")
        self.base.pulse(return_direction, settings["standoff_turn_speed"], settings["standoff_turn_seconds"], "exp2_standoff_restore")

    def step(self, settings):
        if self.context.elapsed() >= float(settings["timeout_seconds"]):
            self.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="exp2 side docking timeout")
        if self.context.increment_step() > int(settings["max_steps"]):
            self.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="exp2 side docking max steps")

        frame = self.depth.read_frame()
        if frame is None:
            self.base.stop()
            return StepOutcome(MissionEvent.FAIL, reason="exp2 camera frame unavailable")
        observation = self.detector.detect(frame)
        if not observation or not observation.get("found"):
            stable = 0
            self.context.state_data["stable_frames"] = stable
            lost = int(self.context.state_data.get("lost_frames", 0)) + 1
            self.context.state_data["lost_frames"] = lost
            self.base.stop()
            if lost >= int(settings.get("lost_frame_limit", 8)):
                return StepOutcome(MissionEvent.TARGET_MISSING, observation, "exp2 side tag lost")
            time.sleep(float(self.config.get("camera.observation_pause_seconds", 0.1)))
            return StepOutcome(observation=observation)

        self.context.state_data["lost_frames"] = 0
        center_error = float(observation.get("error_x", 0.0))
        perspective_error = float(observation.get("vertical_edge_error", 0.0))
        corner_error = float(observation.get("max_corner_angle_error_deg", 90.0))
        height_norm = float(observation.get("bbox_height_norm", 0.0))
        print(
            "[exp2] center_error={:.3f} perspective_error={:.3f} corner_error_deg={:.2f} height_norm={:.3f}".format(
                center_error, perspective_error, corner_error, height_norm
            )
        )

        if abs(perspective_error) > float(settings["perspective_tolerance"]):
            self.context.state_data["stable_frames"] = 0
            direction = self._turn_for_signed_error(
                perspective_error,
                settings["perspective_positive_turn_direction"],
                settings["turn_speed"],
                "exp2_parallel_align",
            )
            print("[exp2] parallel correction direction={}".format(direction))
            return StepOutcome(observation=observation)

        if abs(center_error) > float(settings["center_tolerance_norm"]):
            self.context.state_data["stable_frames"] = 0
            direction = settings["center_positive_drive_direction"] if center_error > 0.0 else self._opposite(settings["center_positive_drive_direction"])
            self.base.start_motion(direction, float(settings["drive_speed"]), "exp2_center_align")
            print("[exp2] longitudinal correction direction={}".format(direction))
            return StepOutcome(observation=observation)

        self.base.stop()

        if bool(settings.get("standoff_correction_enabled", False)):
            target_height = float(settings["target_height_norm"])
            height_tolerance = float(settings["height_tolerance_norm"])
            if height_norm < target_height - height_tolerance:
                self.context.state_data["stable_frames"] = 0
                self._standoff_shift(True, settings)
                print("[exp2] standoff correction=toward_bin")
                return StepOutcome(observation=observation)
            if height_norm > target_height + height_tolerance:
                self.context.state_data["stable_frames"] = 0
                self._standoff_shift(False, settings)
                print("[exp2] standoff correction=away_from_bin")
                return StepOutcome(observation=observation)

        if corner_error > float(settings["corner_angle_tolerance_deg"]):
            self.context.state_data["stable_frames"] = 0
            self.base.stop()
            return StepOutcome(observation=observation, reason="exp2 corner geometry not stable")

        self.base.stop()
        stable = int(self.context.state_data.get("stable_frames", 0)) + 1
        self.context.state_data["stable_frames"] = stable
        print("[exp2] stable frame {}/{}".format(stable, int(settings["stable_frames"])))
        if stable >= int(settings["stable_frames"]):
            return StepOutcome(MissionEvent.TARGET_STABLE, observation)
        return StepOutcome(observation=observation)
