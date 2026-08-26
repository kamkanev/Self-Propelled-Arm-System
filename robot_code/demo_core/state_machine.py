from __future__ import print_function

import os
import time
import traceback

import cv2

from .robot_control import ArmController, BaseController
from .perception import AprilTagBinDetector, CanDetector, DepthSensor
from .fsm_types import MissionContext, MissionEvent, MissionState, TargetType
from .navigation import StepOutcome, TargetNavigator
from .tangentbug import DepthTangentBugPlanner


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
        (MissionState.PLANNING, MissionEvent.TARGET_CACHED): MissionState.VERIFY_TARGET,
        (MissionState.PLANNING, MissionEvent.TARGET_REQUIRED): MissionState.SEARCHING,
        (MissionState.PLANNING, MissionEvent.MISSION_COMPLETE): MissionState.DONE,
        (MissionState.VERIFY_TARGET, MissionEvent.TARGET_FOUND): MissionState.ALIGNING,
        (MissionState.VERIFY_TARGET, MissionEvent.TARGET_MISSING): MissionState.SEARCHING,
        (MissionState.SEARCHING, MissionEvent.TARGET_FOUND): MissionState.ALIGNING,
        (MissionState.SEARCHING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.ALIGNING, MissionEvent.TARGET_ALIGNED): MissionState.APPROACHING,
        (MissionState.ALIGNING, MissionEvent.TARGET_MISSING): MissionState.SEARCHING,
        (MissionState.ALIGNING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.APPROACHING, MissionEvent.TARGET_REACHED): MissionState.FINAL_VERIFY,
        (MissionState.APPROACHING, MissionEvent.TARGET_MISSING): MissionState.SEARCHING,
        (MissionState.APPROACHING, MissionEvent.OBSTACLE_FOUND): MissionState.AVOIDING,
        (MissionState.APPROACHING, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
        (MissionState.FINAL_VERIFY, MissionEvent.TARGET_STABLE): MissionState.FINALIZING,
        (MissionState.FINAL_VERIFY, MissionEvent.TARGET_MISSING): MissionState.SEARCHING,
        (MissionState.FINAL_VERIFY, MissionEvent.TIMEOUT): MissionState.INTERMEDIATE,
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
        self.state = MissionState.IDLE
        self.stop_requested = False
        self.pause_requested = False
        self.previous_state = None
        self.navigator = TargetNavigator(
            config,
            self.context,
            self.services.base,
            self.services.depth,
            self.services.can_detector,
            self.services.bin_detector,
        )
        self.tangentbug = DepthTangentBugPlanner(config.section("avoidance"))
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

    def request_pause(self):
        self.pause_requested = True
        self.services.base.stop()

    def resume(self):
        self.pause_requested = False

    def interrupt_point(self):
        if self.stop_requested:
            raise RuntimeError("stop requested")
        while self.pause_requested:
            self.services.base.stop()
            time.sleep(0.25)
            if self.stop_requested:
                raise RuntimeError("stop requested")

    def _pose(self, name):
        return self.services.arm.pose(name)

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
        if not self.context.grabbed and self.context.completed_pickups >= max_pickups:
            return StepOutcome(MissionEvent.MISSION_COMPLETE)
        target = TargetType.BIN if self.context.grabbed else TargetType.CAN
        self.context.target_type = target
        if target.value in self.context.vague_map:
            return StepOutcome(MissionEvent.TARGET_CACHED)
        return StepOutcome(MissionEvent.TARGET_REQUIRED)

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
        return outcome

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
            self.services.base.pulse("left", settings["turn_speed"], settings["turn_pulse_seconds"], "tangentbug_probe")
        elif plan.action == "forward":
            self.services.base.pulse("forward", settings["forward_speed"], settings["forward_pulse_seconds"], "tangentbug_forward")
        else:
            self.services.base.pulse(plan.action, settings["turn_speed"], settings["turn_pulse_seconds"], "tangentbug_turn")
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
        targets = self._pose("arm_down")
        if not self.services.arm.wait_for_positions(targets, "arm_down"):
            return StepOutcome(MissionEvent.FAIL, reason="arm_down did not settle")
        push = arm["push"]
        if float(push["speed"]) > 0.0 and float(push["seconds"]) > 0.0:
            self.services.base.pulse("forward", push["speed"], push["seconds"], "pickup_push")
        if float(push.get("post_lock_seconds", 0.0)) > 0.0 and arm_is_real:
            time.sleep(float(push["post_lock_seconds"]))
        self._pose("grab")
        self._pose("carry")
        if bool(arm.get("verify_enabled", False)) and not self.config.get("runtime.dry_run.arm", True):
            if not self.services.depth.grab_verified():
                return StepOutcome(MissionEvent.FAIL, reason="pickup verification failed")
        self.context.mark_pickup()
        self.context.forget_target(TargetType.CAN)
        print("[mission] completed_pickups={}".format(self.context.completed_pickups))
        return StepOutcome(MissionEvent.FINALIZED)

    def _run_release_sequence(self):
        self._pose("release")
        self._pose("safe_home")
        self.context.mark_release()
        self.context.forget_target(TargetType.BIN)
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
        if self.state == MissionState.ALIGNING:
            return self._handle_aligning_state()
        if self.state == MissionState.APPROACHING:
            return self._handle_approaching_state()
        if self.state == MissionState.AVOIDING:
            return self._handle_avoiding_state()
        if self.state == MissionState.FINAL_VERIFY:
            return self._handle_final_verify_state()
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
        print("[fsm] stop_all")
        self.services.base.stop()
        try:
            self._pose("safe_home")
        except Exception as exc:
            print("[fsm] safe_home cleanup failed: {}".format(exc))
        self.services.depth.stop()

    def run(self, max_ticks=10000):
        self.stop_requested = False
        self.pause_requested = False
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
        except Exception as exc:
            self.context.last_error = str(exc)
            if self.state != MissionState.FAILED:
                print("[fsm] unhandled error: {}".format(exc))
                traceback.print_exc()
                self.state = MissionState.FAILED
            return False
        finally:
            self.stop_all()
