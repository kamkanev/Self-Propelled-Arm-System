from __future__ import print_function

import os
import threading
import time
import traceback

import cv2

from .robot_control import ArmController, BaseController
from .perception import AprilTagBinDetector, CanDetector, DepthSensor
from .fsm_types import MissionContext, MissionEvent, MissionState, TargetType
from .navigation import BinSideDockingNavigator, StepOutcome, TargetNavigator
from .tangentbug import DepthTangentBugPlanner
from .vague_map import CommandOdometry, Point2D, VagueMap, VagueMapNavigator


class StopRequested(Exception):
    """Cooperative mission stop; this is not a mission failure."""


class RobotComponents(object):
    def __init__(self, base, arm, depth, can_detector, bin_detector):
        self.base = base
        self.arm = arm
        self.depth = depth
        self.can_detector = can_detector
        self.bin_detector = bin_detector

    @classmethod
    def from_config(cls, config):
        return cls(
            BaseController(config),
            ArmController(config),
            DepthSensor(config),
            CanDetector(config),
            AprilTagBinDetector(config),
        )


class DemoStateMachine(object):
    """Explicit event-driven implementation of the design-state machine."""

    TRANSITIONS = {
        (MissionState.IDLE, MissionEvent.START): MissionState.INITIALIZING,
        (MissionState.INITIALIZING, MissionEvent.INITIALIZED): MissionState.PLANNING,
        (MissionState.PLANNING, MissionEvent.TARGET_REQUIRED): MissionState.SEARCHING,
        (MissionState.PLANNING, MissionEvent.PATROL_REQUIRED): MissionState.PATROLLING,
        (MissionState.PLANNING, MissionEvent.MAP_TARGET_AVAILABLE): MissionState.MAP_NAVIGATING,
        (MissionState.PLANNING, MissionEvent.MISSION_COMPLETE): MissionState.DONE,
        (MissionState.VERIFY_TARGET, MissionEvent.TARGET_FOUND): MissionState.ALIGNING,
        (MissionState.VERIFY_TARGET, MissionEvent.TARGET_MISSING): MissionState.SEARCHING,
        (MissionState.SEARCHING, MissionEvent.TARGET_FOUND): MissionState.ALIGNING,
        (MissionState.SEARCHING, MissionEvent.REPLAN): MissionState.PLANNING,
        (MissionState.SEARCHING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.PATROLLING, MissionEvent.TARGET_FOUND): MissionState.ALIGNING,
        (MissionState.PATROLLING, MissionEvent.PATROL_COMPLETE): MissionState.PLANNING,
        (MissionState.PATROLLING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.MAP_NAVIGATING, MissionEvent.TARGET_FOUND): MissionState.ALIGNING,
        (MissionState.MAP_NAVIGATING, MissionEvent.MAP_DESTINATION_REACHED): MissionState.SEARCHING,
        (MissionState.MAP_NAVIGATING, MissionEvent.REPLAN): MissionState.PLANNING,
        (MissionState.MAP_NAVIGATING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.ALIGNING, MissionEvent.TARGET_ALIGNED): MissionState.APPROACHING,
        (MissionState.ALIGNING, MissionEvent.TARGET_MISSING): MissionState.SEARCHING,
        (MissionState.ALIGNING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.APPROACHING, MissionEvent.TARGET_REACHED): MissionState.FINAL_VERIFY,
        (MissionState.APPROACHING, MissionEvent.TARGET_MISSING): MissionState.SEARCHING,
        (MissionState.APPROACHING, MissionEvent.OBSTACLE_FOUND): MissionState.AVOIDING,
        (MissionState.APPROACHING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.FINAL_VERIFY, MissionEvent.TARGET_STABLE): MissionState.FINALIZING,
        (MissionState.FINAL_VERIFY, MissionEvent.SIDE_DOCK_REQUIRED): MissionState.BIN_SIDE_DOCKING,
        (MissionState.FINAL_VERIFY, MissionEvent.TARGET_MISSING): MissionState.SEARCHING,
        (MissionState.FINAL_VERIFY, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.BIN_SIDE_DOCKING, MissionEvent.TARGET_STABLE): MissionState.FINALIZING,
        (MissionState.BIN_SIDE_DOCKING, MissionEvent.TARGET_MISSING): MissionState.INTERMEDIATE,
        (MissionState.BIN_SIDE_DOCKING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.AVOIDING, MissionEvent.PATH_CLEAR): MissionState.VERIFY_TARGET,
        (MissionState.AVOIDING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.FINALIZING, MissionEvent.FINALIZED): MissionState.PLANNING,
        (MissionState.INTERMEDIATE, MissionEvent.RETRY): MissionState.PLANNING,
        (MissionState.INTERMEDIATE, MissionEvent.RETRY_EXHAUSTED): MissionState.FAILED,
    }

    def __init__(self, config, services=None, context=None):
        self.config = config
        self.services = services or RobotComponents.from_config(config)
        self.context = context or MissionContext()
        self.map_enabled = bool(config.get("vague_map.enabled", False))
        self.vague_map = None
        self.map_navigator = None
        if self.map_enabled:
            self.vague_map = self.context.vague_map or VagueMap(config.section("vague_map"))
            self.context.vague_map = self.vague_map
            odometry = CommandOdometry(self.vague_map, config.section("vague_map")["odometry"])
            if hasattr(self.services.base, "attach_motion_tracker"):
                self.services.base.attach_motion_tracker(odometry)
            self.map_navigator = VagueMapNavigator(
                self.vague_map,
                self.services.base,
                config.section("vague_map")["navigation"],
            )
        self.state = MissionState.IDLE
        self.stop_requested = False
        self.pause_requested = False
        self._cleanup_lock = threading.Lock()
        self._cleanup_complete = False
        self.previous_state = None
        self.navigator = TargetNavigator(
            config,
            self.context,
            self.services.base,
            self.services.depth,
            self.services.can_detector,
            self.services.bin_detector,
            frame_observer=self._observe_navigation_frame,
        )
        self.tangentbug = DepthTangentBugPlanner(config.section("avoidance"))
        self.side_docking = BinSideDockingNavigator(
            config,
            self.context,
            self.services.base,
            self.services.depth,
            self.services.bin_detector,
        )
        self.exp2_side_docked = False
        self.context.begin_state(self.state)

    def transition(self, event, reason=None):
        if event == MissionEvent.FAIL:
            next_state = MissionState.FAILED
        else:
            key = (self.state, event)
            if key not in self.TRANSITIONS:
                raise RuntimeError("invalid FSM transition: {} + {}".format(self.state.value, event.value))
            next_state = self.TRANSITIONS[key]
        if reason:
            self.context.last_error = reason
        self.context.finish_state(event)
        print("[fsm] {} --{}--> {}{}".format(
            self.state.value,
            event.value,
            next_state.value,
            " reason={}".format(reason) if reason else "",
        ))
        self.previous_state = self.state
        self.state = next_state
        self.context.begin_state(self.state)

    def request_stop(self):
        self.stop_requested = True
        self.pause_requested = False
        self.services.base.stop()
        if hasattr(self.services.arm, "cancel_motion"):
            self.services.arm.cancel_motion()

    def request_pause(self):
        self.pause_requested = True
        self.services.base.stop()

    def resume(self):
        self.pause_requested = False

    def interrupt_point(self):
        if self.stop_requested:
            raise StopRequested("stop requested")
        while self.pause_requested:
            self.services.base.stop()
            time.sleep(0.25)
            if self.stop_requested:
                raise StopRequested("stop requested")

    def _pose(self, name):
        while True:
            targets = self.services.arm.pose(name)
            if not getattr(self.services.arm, "last_pose_interrupted", False):
                return targets
            print("[fsm] arm pose {} paused after interruption; Resume retries the pose".format(name))
            self.interrupt_point()

    def _wait_for_pose(self, name, targets):
        while True:
            if self.services.arm.wait_for_positions(targets, name):
                return True
            cancelled = (
                hasattr(self.services.arm, "motion_cancelled")
                and self.services.arm.motion_cancelled()
            )
            if not cancelled:
                return False
            print("[fsm] arm wait {} interrupted; Resume retries the pose".format(name))
            self.interrupt_point()
            targets = self._pose(name)

    def start_camera(self):
        self.services.depth.start()

    def release_camera(self):
        self.services.depth.stop()

    def load_detectors(self, reload_models=False):
        if reload_models:
            self.services.can_detector.reset()
            self.services.bin_detector.detector = None
            self.services.bin_detector.aruco_dict = None
        self.services.can_detector.load()
        self.services.bin_detector.load()
        return True

    def _handle_initializing_state(self):
        self.services.depth.start()
        self._pose("safe_home")
        self.services.base.stop()
        return StepOutcome(MissionEvent.INITIALIZED)

    def _handle_planning_state(self):
        max_pickups = int(self.config.get("runtime.max_pickups", 1))
        if not self.context.grabbed and max_pickups > 0 and self.context.completed_pickups >= max_pickups:
            return StepOutcome(MissionEvent.MISSION_COMPLETE)

        if self.map_enabled:
            if self.context.grabbed:
                self.context.target_type = TargetType.BIN
                return StepOutcome(MissionEvent.MAP_TARGET_AVAILABLE)

            mapped_can = self.vague_map.nearest_can()
            if mapped_can is not None:
                self.context.target_type = TargetType.CAN
                return StepOutcome(MissionEvent.MAP_TARGET_AVAILABLE)

            if not self.vague_map.initial_scan_complete:
                self.context.target_type = TargetType.CAN
                return StepOutcome(MissionEvent.TARGET_REQUIRED)
            if self.vague_map.patrol_complete:
                return StepOutcome(MissionEvent.MISSION_COMPLETE)
            self.context.target_type = TargetType.CAN
            return StepOutcome(MissionEvent.PATROL_REQUIRED)

        target = TargetType.BIN if self.context.grabbed else TargetType.CAN
        self.context.target_type = target
        return StepOutcome(MissionEvent.TARGET_REQUIRED)

    def _map_can_detections(self, frame, detections):
        if not self.map_enabled or frame is None or not detections:
            return []
        depth_map = self.services.depth.depth_map_frame(frame)
        if depth_map is None:
            print("[map] can mapping skipped reason=depth_unavailable")
            return []
        height, width = frame.shape[:2]
        settings = self.config.section("vague_map")
        mapped = []
        for index, observation in enumerate(detections):
            stats = self.services.depth.observe_center_depth_map(
                "map_can_{}_depth".format(index),
                depth_map,
                observation.get("center_x"),
                observation.get("center_y"),
                width,
                height,
                float(settings.get("can_depth_roi_width", 0.12)),
                float(settings.get("can_depth_roi_height", 0.18)),
            )
            if not stats:
                continue
            position = self.vague_map.estimate_can_position(
                observation,
                stats.get("mean"),
                height,
            )
            if position is None:
                continue
            mapped_can = self.vague_map.remember_can(
                position,
                observation.get("confidence", 0.0),
                observation.get("timestamp"),
            )
            if mapped_can is not None:
                mapped.append(mapped_can)
        return mapped

    def _observe_navigation_frame(self, target_type, frame):
        if not self.map_enabled or not self.context.grabbed or target_type != TargetType.BIN:
            return
        detections = self.services.can_detector.detect_all(frame)
        self._map_can_detections(frame, detections)

    def _best_can_observation(self, detections):
        accepted = [
            observation
            for observation in detections
            if self.navigator._accepted(TargetType.CAN, observation, tracking=True)
        ]
        if not accepted:
            return None
        return max(accepted, key=lambda observation: float(observation.get("confidence", 0.0)))

    def _handle_verify_target_state(self):
        observation = self.navigator.detect(self.context.target_type)
        if self.navigator._accepted(self.context.target_type, observation, tracking=True):
            self.context.remember_target(self.context.target_type, observation)
            return StepOutcome(MissionEvent.TARGET_FOUND, observation)
        self.context.forget_target(self.context.target_type)
        return StepOutcome(MissionEvent.TARGET_MISSING, observation)

    def _handle_searching_state(self):
        outcome = self.navigator.search_step(self.context.target_type)
        if outcome.event == MissionEvent.TARGET_FOUND:
            self.context.remember_target(self.context.target_type, outcome.observation)
            if self.map_enabled and self.context.target_type == TargetType.CAN:
                self.vague_map.initial_scan_complete = True
        elif self.map_enabled and self.context.target_type == TargetType.CAN and outcome.event == MissionEvent.TIMEOUT:
            self.vague_map.initial_scan_complete = True
            if self.vague_map.selected_can_id is not None:
                self.vague_map.remove_can(self.vague_map.selected_can_id, "not_found_near_cached_position")
            return StepOutcome(MissionEvent.REPLAN, reason="can search completed without target")
        return outcome

    def _map_state_timed_out(self):
        navigation = self.config.section("vague_map")["navigation"]
        if self.context.increment_step() > int(navigation.get("max_steps", 1000)):
            return True
        return self.context.elapsed() >= float(navigation.get("timeout_seconds", 30.0))

    def _handle_patrolling_state(self):
        if self._map_state_timed_out():
            self.services.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="patrol navigation timeout")
        frame = self.services.depth.read_frame()
        if frame is None:
            return StepOutcome(MissionEvent.TIMEOUT, reason="patrol camera frame unavailable")
        detections = self.services.can_detector.detect_all(frame)
        self._map_can_detections(frame, detections)
        best = self._best_can_observation(detections)
        if best is not None:
            self.services.base.stop()
            self.context.remember_target(TargetType.CAN, best)
            self.vague_map.nearest_can()
            return StepOutcome(MissionEvent.TARGET_FOUND, best)
        if self.map_navigator.patrol_step():
            return StepOutcome(MissionEvent.PATROL_COMPLETE)
        return StepOutcome()

    def _map_destination(self):
        if self.context.target_type == TargetType.BIN:
            return Point2D(
                self.vague_map.bin_docking_pose.x_m,
                self.vague_map.bin_docking_pose.y_m,
            ), "bin_docking"
        mapped_can = self.vague_map.known_cans.get(self.vague_map.selected_can_id)
        if mapped_can is None:
            return None, None
        return mapped_can.position, "can_{}".format(mapped_can.can_id)

    def _handle_map_navigating_state(self):
        if self._map_state_timed_out():
            self.services.base.stop()
            if self.context.target_type == TargetType.CAN and self.vague_map.selected_can_id is not None:
                self.vague_map.remove_can(self.vague_map.selected_can_id, "map_navigation_timeout")
                return StepOutcome(MissionEvent.REPLAN, reason="cached can navigation timeout")
            return StepOutcome(MissionEvent.TIMEOUT, reason="map navigation timeout")

        frame = self.services.depth.read_frame()
        if frame is None:
            return StepOutcome(MissionEvent.TIMEOUT, reason="map navigation camera frame unavailable")
        if self.context.target_type == TargetType.CAN:
            detections = self.services.can_detector.detect_all(frame)
            self._map_can_detections(frame, detections)
            observation = self._best_can_observation(detections)
        else:
            observation = self.navigator.detect(TargetType.BIN, frame)
            if not self.navigator._accepted(TargetType.BIN, observation, tracking=True):
                observation = None
        if observation is not None:
            self.services.base.stop()
            self.context.remember_target(self.context.target_type, observation)
            return StepOutcome(MissionEvent.TARGET_FOUND, observation)

        destination, label = self._map_destination()
        if destination is None:
            return StepOutcome(MissionEvent.REPLAN, reason="map target no longer available")
        if self.map_navigator.step_toward(destination, label):
            return StepOutcome(MissionEvent.MAP_DESTINATION_REACHED)
        return StepOutcome()

    def _handle_aligning_state(self):
        outcome = self.navigator.align_step(self.context.target_type)
        if outcome.observation and outcome.observation.get("found"):
            self.context.remember_target(self.context.target_type, outcome.observation)
        return outcome

    def _handle_approaching_state(self):
        outcome = self.navigator.approach_step(self.context.target_type)
        if outcome.observation and outcome.observation.get("found"):
            self.context.remember_target(self.context.target_type, outcome.observation)
        return outcome

    def _handle_final_verify_state(self):
        outcome = self.navigator.final_verify_step(self.context.target_type)
        if outcome.observation and outcome.observation.get("found"):
            self.context.remember_target(self.context.target_type, outcome.observation)
        exp2_enabled = bool(self.config.get("navigation.bin.side_docking.experimental.enabled", False))
        if (
            self.map_enabled
            and self.context.target_type == TargetType.BIN
            and outcome.event == MissionEvent.TARGET_STABLE
            and not exp2_enabled
        ):
            self.vague_map.set_robot_pose(self.vague_map.bin_docking_pose, "bin_visual_docking")
        if (
            self.context.target_type == TargetType.BIN
            and outcome.event == MissionEvent.TARGET_STABLE
            and exp2_enabled
        ):
            return StepOutcome(MissionEvent.SIDE_DOCK_REQUIRED, outcome.observation)
        return outcome

    def _handle_bin_side_docking_state(self):
        settings = self.config.get("navigation.bin.side_docking.experimental")
        if not self.context.state_data.get("entry_complete", False):
            self.services.base.stop()
            print("[exp2] phase=side_view_pose")
            targets = self._pose("side_view_grabbing")
            if not self._wait_for_pose("side_view_grabbing", targets):
                return StepOutcome(MissionEvent.FAIL, reason="exp2 side-view arm pose did not settle")
            self.interrupt_point()
            print("[exp2] phase=supportive_base_turn")
            self.services.base.pulse(
                settings["supportive_turn_direction"],
                settings["supportive_turn_speed"],
                settings["supportive_turn_seconds"],
                "exp2_supportive_base_turn",
            )
            self.context.state_data["entry_complete"] = True
            print("[exp2] phase=geometry_alignment_ready")
            return StepOutcome()
        outcome = self.side_docking.step(settings)
        if outcome.observation and outcome.observation.get("found"):
            self.context.remember_target(TargetType.BIN, outcome.observation)
        if outcome.event == MissionEvent.TARGET_STABLE:
            self.exp2_side_docked = True
        elif outcome.event in (MissionEvent.TARGET_MISSING, MissionEvent.TIMEOUT, MissionEvent.FAIL):
            self.services.base.stop()
            self._pose("carry")
            self.interrupt_point()
            opposite = {
                "left": "right",
                "right": "left",
                "forward": "backward",
                "backward": "forward",
            }[settings["supportive_turn_direction"]]
            self.services.base.pulse(
                opposite,
                settings["supportive_turn_speed"],
                settings["supportive_turn_seconds"],
                "exp2_abort_reset_heading",
            )
            print("[exp2] side docking aborted; forward camera pose and heading restored")
        return outcome

    def _scripted_avoidance(self):
        settings = self.config.section("avoidance")
        self.services.base.pulse("left", settings["turn_speed"], settings["turn_pulse_seconds"], "avoid_left")
        self.services.base.pulse("forward", settings["forward_speed"], settings["forward_pulse_seconds"], "avoid_forward")
        self.services.base.pulse("right", settings["turn_speed"], settings["turn_pulse_seconds"], "avoid_rejoin")
        self.context.obstacle_found = False
        return StepOutcome(MissionEvent.PATH_CLEAR)

    def _tangentbug_avoidance(self):
        settings = self.config.section("avoidance")
        if self.context.increment_step() > int(settings.get("max_steps", 20)):
            self.services.base.stop()
            return StepOutcome(MissionEvent.TIMEOUT, reason="tangentbug max steps")
        frame = self.services.depth.read_frame()
        depth_map = self.services.depth.depth_map_frame(frame)
        target_error = 0.0
        if self.context.last_observation:
            target_error = float(self.context.last_observation.get("error_x", 0.0))
        plan = self.tangentbug.plan(depth_map, target_error)
        print("[tangentbug] {}".format(plan.as_dict()))
        debug_path = self.config.resolve_path(settings.get("debug_overlay_path"))
        if debug_path and frame is not None:
            parent = os.path.dirname(debug_path)
            if parent and not os.path.exists(parent):
                os.makedirs(parent)
            cv2.imwrite(debug_path, self.tangentbug.draw_debug(frame, plan))
        if plan.path_clear:
            self.services.base.stop()
            self.context.obstacle_found = False
            return StepOutcome(MissionEvent.PATH_CLEAR)
        if plan.action == "blocked":
            self.services.base.start_motion("left", settings["turn_speed"], "tangentbug_probe")
        elif plan.action == "forward":
            self.services.base.start_motion("forward", settings["forward_speed"], "tangentbug_forward")
        else:
            self.services.base.start_motion(plan.action, settings["turn_speed"], "tangentbug_turn")
        return StepOutcome()

    def _handle_avoiding_state(self):
        strategy = self.config.get("avoidance.strategy", "disabled")
        if strategy == "scripted":
            return self._scripted_avoidance()
        if strategy == "tangentbug_depth":
            return self._tangentbug_avoidance()
        self.context.obstacle_found = False
        return StepOutcome(MissionEvent.PATH_CLEAR)

    def _run_pickup_sequence(self):
        arm = self.config.section("arm")
        delay = float(arm.get("pickup_start_delay_seconds", 0.0))
        arm_is_real = not self.config.get("runtime.dry_run.arm", True)
        if delay > 0 and arm_is_real:
            time.sleep(delay)
        self.interrupt_point()
        targets = self._pose("arm_down")
        self.interrupt_point()
        if not self._wait_for_pose("arm_down", targets):
            return StepOutcome(MissionEvent.FAIL, reason="arm_down did not settle")
        self.interrupt_point()
        push = arm["push"]
        if float(push["speed"]) > 0.0 and float(push["seconds"]) > 0.0:
            self.services.base.pulse("forward", push["speed"], push["seconds"], "pickup_push")
        if float(push.get("post_lock_seconds", 0.0)) > 0.0 and arm_is_real:
            time.sleep(float(push["post_lock_seconds"]))
        self.interrupt_point()
        self._pose("grab")
        self.interrupt_point()
        self._pose("carry")
        if bool(arm.get("verify_enabled", False)) and not self.config.get("runtime.dry_run.arm", True):
            if not self.services.depth.grab_verified():
                return StepOutcome(MissionEvent.FAIL, reason="pickup verification failed")
        self.context.mark_pickup()
        if hasattr(self.services.can_detector, "mark_dry_run_target_consumed"):
            self.services.can_detector.mark_dry_run_target_consumed()
        if self.map_enabled:
            self.vague_map.remove_cans_near_robot()
        self.context.forget_target(TargetType.CAN)
        print("[mission] completed_pickups={}".format(self.context.completed_pickups))
        return StepOutcome(MissionEvent.FINALIZED)

    def _run_release_sequence(self):
        if self.exp2_side_docked:
            print("[exp2] phase=release_low")
            targets = self._pose("side_view_release_low")
            if not self._wait_for_pose("side_view_release_low", targets):
                return StepOutcome(MissionEvent.FAIL, reason="exp2 release pose did not settle")
            self.interrupt_point()
            print("[exp2] phase=safe_home")
            targets = self._pose("safe_home")
            if not self._wait_for_pose("safe_home", targets):
                return StepOutcome(MissionEvent.FAIL, reason="exp2 safe_home pose did not settle")
            if self.map_enabled:
                pose = self.vague_map.bin_side_docking_pose
                self.vague_map.set_robot_pose(pose, "exp2_side_parked_after_release")
            print("[exp2] phase=side_parked")
        else:
            self._pose("release")
            self.interrupt_point()
            self._pose("safe_home")
        self.context.mark_release()
        self.context.forget_target(TargetType.BIN)
        self.exp2_side_docked = False
        return StepOutcome(MissionEvent.FINALIZED)

    def _handle_finalizing_state(self):
        if self.context.target_type == TargetType.CAN:
            return self._run_pickup_sequence()
        if self.context.target_type == TargetType.BIN:
            return self._run_release_sequence()
        return StepOutcome(MissionEvent.FAIL, reason="finalize without target")

    def _handle_intermediate_state(self):
        failed_state = self.previous_state or MissionState.INTERMEDIATE
        count = self.context.retry(failed_state)
        limit = int(self.config.get("runtime.retry_limit", 2))
        self.context.forget_target(self.context.target_type)
        if count <= limit:
            return StepOutcome(MissionEvent.RETRY, reason="retry {}/{} after {}".format(count, limit, failed_state.value))
        return StepOutcome(MissionEvent.RETRY_EXHAUSTED, reason="retry limit exceeded after {}".format(failed_state.value))

    def _evaluate_current_state(self):
        self.interrupt_point()
        if self.state == MissionState.IDLE:
            return StepOutcome(MissionEvent.START)
        if self.state == MissionState.INITIALIZING:
            return self._handle_initializing_state()
        if self.state == MissionState.PLANNING:
            return self._handle_planning_state()
        if self.state == MissionState.VERIFY_TARGET:
            return self._handle_verify_target_state()
        if self.state == MissionState.SEARCHING:
            return self._handle_searching_state()
        if self.state == MissionState.PATROLLING:
            return self._handle_patrolling_state()
        if self.state == MissionState.MAP_NAVIGATING:
            return self._handle_map_navigating_state()
        if self.state == MissionState.ALIGNING:
            return self._handle_aligning_state()
        if self.state == MissionState.APPROACHING:
            return self._handle_approaching_state()
        if self.state == MissionState.AVOIDING:
            return self._handle_avoiding_state()
        if self.state == MissionState.FINAL_VERIFY:
            return self._handle_final_verify_state()
        if self.state == MissionState.BIN_SIDE_DOCKING:
            return self._handle_bin_side_docking_state()
        if self.state == MissionState.FINALIZING:
            return self._handle_finalizing_state()
        if self.state == MissionState.INTERMEDIATE:
            return self._handle_intermediate_state()
        return StepOutcome()

    def step_once(self):
        if self.state in (MissionState.DONE, MissionState.FAILED):
            return StepOutcome()
        outcome = self._evaluate_current_state()
        if outcome.event is not None:
            self.transition(outcome.event, outcome.reason)
        return outcome

    def stop_all(self):
        with self._cleanup_lock:
            if self._cleanup_complete:
                print("[fsm] stop_all already complete")
                return
            print("[fsm] stop_all owner={}".format(threading.current_thread().name))
            try:
                self.services.base.stop()
                try:
                    self.services.arm.pose("safe_home")
                except Exception as exc:
                    print("[fsm] safe_home cleanup failed: {}".format(exc))
                self.services.depth.stop()
            finally:
                self._cleanup_complete = True

    def run(self, max_ticks=10000, cleanup=True):
        self.pause_requested = False
        with self._cleanup_lock:
            self._cleanup_complete = False
        try:
            for _ in range(int(max_ticks)):
                if self.state in (MissionState.DONE, MissionState.FAILED):
                    break
                self.step_once()
                time.sleep(float(self.config.get("runtime.loop_pause_seconds", 0.02)))
            if self.state not in (MissionState.DONE, MissionState.FAILED):
                self.transition(MissionEvent.FAIL, "runtime max ticks exceeded")
            print("[fsm] terminal={} context={}".format(self.state.value, self.context.snapshot()))
            return self.state == MissionState.DONE
        except StopRequested:
            self.context.last_error = None
            print("[fsm] stop requested; ending run without mission failure")
            return False
        except Exception as exc:
            self.context.last_error = str(exc)
            if self.state != MissionState.FAILED:
                print("[fsm] unhandled error: {}".format(exc))
                traceback.print_exc()
                self.state = MissionState.FAILED
            return False
        finally:
            if cleanup:
                self.stop_all()
