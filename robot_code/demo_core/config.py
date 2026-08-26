from __future__ import print_function

import copy
import json
import os
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PACKAGE_ROOT / "config.json"


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
        for target_name in ("can", "bin"):
            target = self.data["navigation"][target_name]
            if float(target["search"]["timeout_seconds"]) <= 0:
                raise ValueError("{}.search.timeout_seconds must be positive".format(target_name))
            if int(target["align"]["max_steps"]) <= 0:
                raise ValueError("{}.align.max_steps must be positive".format(target_name))
            if float(target["approach"]["timeout_seconds"]) <= 0:
                raise ValueError("{}.approach.timeout_seconds must be positive".format(target_name))
        strategy = self.get("avoidance.strategy", "disabled")
        if strategy not in ("disabled", "scripted", "tangentbug_depth"):
            raise ValueError("unsupported avoidance.strategy: {}".format(strategy))
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
