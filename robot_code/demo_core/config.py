from __future__ import print_function

import copy
import json
import math
import os
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PACKAGE_ROOT / "config.json"


BASE_SPEED_REFERENCE_PATHS = {
    "navigation.can.search.speed": "turn",
    "navigation.can.align.speed": "turn",
    "navigation.can.approach.experimental_continuous.speed": "linear",
    "navigation.can.approach.speed": "linear",
    "navigation.can.approach.steering_speed": "turn",
    "navigation.can.near_align.speed": "turn",
    "navigation.bin.search.speed": "turn",
    "navigation.bin.align.speed": "turn",
    "navigation.bin.approach.speed": "linear",
    "navigation.bin.approach.steering_speed": "turn",
    "navigation.bin.side_docking.experimental.supportive_turn_speed": "turn",
    "navigation.bin.side_docking.experimental.drive_speed": "linear",
    "navigation.bin.side_docking.experimental.turn_speed": "turn",
    "navigation.bin.side_docking.experimental.standoff_turn_speed": "turn",
    "navigation.bin.side_docking.experimental.standoff_drive_speed": "linear",
    "vague_map.navigation.turn_speed": "turn",
    "vague_map.navigation.forward_speed": "linear",
    "avoidance.turn_speed": "turn",
    "avoidance.forward_speed": "linear",
    "arm.push.speed": "linear",
}


def _deep_merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _require(mapping, path):
    value = mapping
    for key in path.split("."):
        if not isinstance(value, dict) or key not in value:
            raise ValueError("missing config key: {}".format(path))
        value = value[key]
    return value


def _read_json(path):
    with open(str(path), "r") as stream:
        return json.load(stream)


def _set_path(mapping, path, value):
    target = mapping
    keys = path.split(".")
    for key in keys[:-1]:
        target = target[key]
    target[keys[-1]] = value


def _resolve_base_speed_references(data):
    profiles = _require(data, "base_motion_speed_profiles")
    for motion_type in ("linear", "turn"):
        values = _require(profiles, motion_type)
        for level in ("fast", "slow"):
            if float(_require(values, level)) <= 0.0:
                raise ValueError("base_motion_speed_profiles.{}.{} must be positive".format(motion_type, level))
    for path, expected_type in BASE_SPEED_REFERENCE_PATHS.items():
        reference = _require(data, path)
        if not isinstance(reference, str):
            raise ValueError("{} must use a symbolic base speed reference".format(path))
        parts = reference.split(".")
        if len(parts) != 2 or parts[0] != expected_type or parts[1] not in ("fast", "slow"):
            raise ValueError(
                "{} must reference {}.fast or {}.slow; got {}".format(
                    path, expected_type, expected_type, reference
                )
            )
        _set_path(data, path, float(profiles[parts[0]][parts[1]]))
    return data


def _runtime_paths(config_path=None):
    config_path = Path(config_path or CONFIG_PATH).resolve()
    runtime_config = _read_json(config_path)
    paths = runtime_config.get("paths", {})
    root_value = Path(paths.get("project_root", "."))
    project_root = root_value if root_value.is_absolute() else (config_path.parent / root_value).resolve()
    return config_path, runtime_config, paths, project_root


class DemoConfig(object):
    REQUIRED_PATHS = (
        "runtime.dry_run.base",
        "runtime.dry_run.arm",
        "runtime.dry_run.camera",
        "runtime.retry_limit",
        "camera.width",
        "camera.height",
        "detectors.can.model_path",
        "detectors.can.labels_path",
        "detectors.can.confidence_threshold",
        "detectors.bin.tag_id",
        "navigation.can.search",
        "navigation.can.align",
        "navigation.can.approach",
        "navigation.bin.search",
        "navigation.bin.align",
        "navigation.bin.approach",
        "arm.poses.safe_home",
        "arm.poses.arm_down",
        "arm.poses.grab",
        "arm.poses.carry",
        "arm.poses.release",
    )

    def __init__(self, data, config_path, parameters_path, project_root):
        self.data = data
        self.config_path = os.path.abspath(str(config_path))
        self.parameters_path = os.path.abspath(str(parameters_path))
        self.project_root = os.path.abspath(str(project_root))
        self.validate()

    def validate(self):
        for path in self.REQUIRED_PATHS:
            _require(self.data, path)
        command_scales = _require(self.data, "base_motion_command_scales")
        for direction in ("forward", "backward", "left", "right"):
            value = float(_require(command_scales, direction))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("base_motion_command_scales.{} must be positive".format(direction))
        for target_name in ("can", "bin"):
            target = self.data["navigation"][target_name]
            if float(target["search"]["timeout_seconds"]) <= 0:
                raise ValueError("{}.search.timeout_seconds must be positive".format(target_name))
            if int(target["align"]["max_steps"]) <= 0:
                raise ValueError("{}.align.max_steps must be positive".format(target_name))
            if float(target["approach"]["timeout_seconds"]) <= 0:
                raise ValueError("{}.approach.timeout_seconds must be positive".format(target_name))
        experiment = self.get("navigation.can.approach.experimental_continuous", {})
        if bool(experiment.get("enabled", False)):
            if float(experiment.get("speed", 0.0)) <= 0.0:
                raise ValueError("navigation.can.approach.experimental_continuous.speed must be positive")
            if float(experiment.get("duration_seconds", 0.0)) <= 0.0:
                raise ValueError("navigation.can.approach.experimental_continuous.duration_seconds must be positive")
        exp2 = self.get("navigation.bin.side_docking.experimental", {})
        if bool(exp2.get("enabled", False)):
            for path in ("arm.poses.side_view_grabbing", "arm.poses.side_view_release_low", "vague_map.bin_side_docking_pose"):
                _require(self.data, path)
            for key in (
                "supportive_turn_speed", "supportive_turn_seconds", "drive_speed",
                "turn_speed", "timeout_seconds",
            ):
                if float(exp2.get(key, 0.0)) <= 0.0:
                    raise ValueError("navigation.bin.side_docking.experimental.{} must be positive".format(key))
            for key in ("stable_frames", "lost_frame_limit", "max_steps"):
                if int(exp2.get(key, 0)) <= 0:
                    raise ValueError("navigation.bin.side_docking.experimental.{} must be positive".format(key))
            valid_directions = ("left", "right", "forward", "backward")
            for key in (
                "supportive_turn_direction", "center_positive_drive_direction",
                "perspective_positive_turn_direction", "standoff_toward_turn_direction",
            ):
                if exp2.get(key) not in valid_directions:
                    raise ValueError("navigation.bin.side_docking.experimental.{} is invalid".format(key))
        strategy = self.get("avoidance.strategy", "disabled")
        if strategy not in ("disabled", "scripted", "tangentbug_depth"):
            raise ValueError("unsupported avoidance.strategy: {}".format(strategy))
        max_pickups = int(self.get("runtime.max_pickups", 1))
        if max_pickups < 0:
            raise ValueError("runtime.max_pickups must be zero or positive")
        if bool(self.get("vague_map.enabled", False)):
            vague_map = _require(self.data, "vague_map")
            for name in (
                "bounds_m",
                "initial_pose",
                "bin_marker_position",
                "bin_docking_pose",
                "patrol_waypoints",
                "odometry",
                "navigation",
                "camera_horizontal_fov_rad",
                "depth_to_distance_scale_m_per_unit",
            ):
                if name not in vague_map:
                    raise ValueError("missing config key: vague_map.{}".format(name))
            bounds = vague_map["bounds_m"]
            if float(bounds["min_x"]) >= float(bounds["max_x"]) or float(bounds["min_y"]) >= float(bounds["max_y"]):
                raise ValueError("vague_map.bounds_m must define a positive area")
            odometry = vague_map["odometry"]
            for key in ("linear_meters_per_speed_second", "linear_slip_factor", "angular_radians_per_speed_second"):
                if float(odometry[key]) <= 0.0:
                    raise ValueError("vague_map.odometry.{} must be positive".format(key))
            navigation = vague_map["navigation"]
            for key in ("turn_speed", "turn_pulse_seconds", "forward_speed", "forward_pulse_seconds", "timeout_seconds"):
                if float(navigation[key]) <= 0.0:
                    raise ValueError("vague_map.navigation.{} must be positive".format(key))
            if int(navigation.get("max_steps", 0)) <= 0:
                raise ValueError("vague_map.navigation.max_steps must be positive")
        elif max_pickups <= 0:
            raise ValueError("runtime.max_pickups must be positive when vague_map is disabled")
        return True

    def section(self, name):
        return self.data[name]

    def target_navigation(self, target_type):
        name = target_type.value if hasattr(target_type, "value") else str(target_type)
        return self.data["navigation"][name]

    def get(self, path, default=None):
        value = self.data
        for key in path.split("."):
            if not isinstance(value, dict) or key not in value:
                return default
            value = value[key]
        return value

    def resolve_path(self, path):
        if not path or os.path.isabs(path):
            return path
        return os.path.abspath(os.path.join(self.project_root, path))


def load_empirical_parameters(path=None, config_path=None):
    _, _, paths, project_root = _runtime_paths(config_path)
    value = Path(path or paths.get("empirical_parameters", "empirical_parameters.json"))
    parameters_path = value if value.is_absolute() else (project_root / value).resolve()
    return _read_json(parameters_path)


def save_empirical_parameters(data, path=None, config_path=None):
    _, _, paths, project_root = _runtime_paths(config_path)
    value = Path(path or paths.get("empirical_parameters", "empirical_parameters.json"))
    parameters_path = value if value.is_absolute() else (project_root / value).resolve()
    with open(str(parameters_path), "w") as stream:
        json.dump(data, stream, indent=2)
        stream.write("\n")
    print("[parameters] saved {}".format(parameters_path))
    return str(parameters_path)


def load_config(parameters_path=None, config_path=None, overrides=None):
    config_path, runtime_config, paths, project_root = _runtime_paths(config_path)
    value = Path(parameters_path or paths.get("empirical_parameters", "empirical_parameters.json"))
    parameters_path = value if value.is_absolute() else (project_root / value).resolve()
    data = _deep_merge(_read_json(parameters_path), runtime_config)
    if overrides:
        data = _deep_merge(data, overrides)
    data = _resolve_base_speed_references(data)
    return DemoConfig(data, config_path, parameters_path, project_root)


_, PROJECT_CONFIG, PROJECT_PATHS, BASE_DIR = _runtime_paths()


def project_path(name, default):
    value = Path(PROJECT_PATHS.get(name, default))
    return value if value.is_absolute() else (BASE_DIR / value).resolve()


EMPIRICAL_PARAMETERS_PATH = project_path("empirical_parameters", "empirical_parameters.json")
ASSETS_DIR = project_path("assets", "assets")
TESTS_DIR = project_path("tests", "tests")
LEGACY_PARAMS_DIR = project_path("legacy_params", "legacy_params")
CAN_MODEL_DIR = project_path("can_model_directory", "assets/models/detectnet_native_can")
APRILTAG_IMAGE_PATH = project_path("apriltag_image", "assets/bin_apriltag_36h11_id_0.png")
LOG_DIR = project_path("logs", "logs")
DIAGNOSTIC_OUTPUT_DIR = project_path("diagnostic_outputs", "diagnostic_outputs")
