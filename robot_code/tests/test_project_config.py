import os
import copy
import json
import tempfile
import unittest

from demo_core.config import (
    ASSETS_DIR,
    APRILTAG_IMAGE_PATH,
    BASE_DIR,
    CAN_MODEL_DIR,
    CONFIG_PATH,
    DIAGNOSTIC_OUTPUT_DIR,
    EMPIRICAL_PARAMETERS_PATH,
    LOG_DIR,
    PROJECT_PATHS,
    LEGACY_PARAMS_DIR,
    TESTS_DIR,
    load_config,
)


class ProjectConfigTest(unittest.TestCase):
    def test_delivery_paths_are_configured_and_rooted(self):
        self.assertTrue(CONFIG_PATH.is_file())
        self.assertEqual(PROJECT_PATHS["project_root"], ".")
        self.assertEqual(EMPIRICAL_PARAMETERS_PATH, BASE_DIR / "empirical_parameters.json")
        self.assertEqual(ASSETS_DIR, BASE_DIR / "assets")
        self.assertEqual(TESTS_DIR, BASE_DIR / "tests")
        self.assertEqual(LEGACY_PARAMS_DIR, BASE_DIR / "legacy_params")
        self.assertEqual(CAN_MODEL_DIR, ASSETS_DIR / "models" / "detectnet_native_can")
        self.assertEqual(APRILTAG_IMAGE_PATH, ASSETS_DIR / "bin_apriltag_36h11_id_0.png")
        self.assertEqual(LOG_DIR, BASE_DIR / "logs")
        self.assertEqual(DIAGNOSTIC_OUTPUT_DIR, BASE_DIR / "diagnostic_outputs")
        self.assertTrue(os.path.isdir(str(ASSETS_DIR)))
        self.assertTrue(APRILTAG_IMAGE_PATH.is_file())
        self.assertTrue((CAN_MODEL_DIR / "can_ssd_mobilenet_v1.onnx").is_file())
        self.assertNotIn("legacy_parameters", PROJECT_PATHS)
        self.assertTrue((LEGACY_PARAMS_DIR / "old_demo_params_reference.json").is_file())

    def test_merge_priority_is_parameters_then_config_then_overrides(self):
        with open(str(EMPIRICAL_PARAMETERS_PATH), "r") as stream:
            parameters = json.load(stream)
        with open(str(CONFIG_PATH), "r") as stream:
            runtime_config = json.load(stream)
        parameters = copy.deepcopy(parameters)
        runtime_config = copy.deepcopy(runtime_config)
        parameters.setdefault("runtime", {}).setdefault("dry_run", {})["arm"] = True
        runtime_config["runtime"]["dry_run"]["arm"] = False
        runtime_config["paths"]["project_root"] = str(BASE_DIR)
        with tempfile.TemporaryDirectory() as directory:
            parameters_path = os.path.join(directory, "parameters.json")
            config_path = os.path.join(directory, "config.json")
            with open(parameters_path, "w") as stream:
                json.dump(parameters, stream)
            with open(config_path, "w") as stream:
                json.dump(runtime_config, stream)
            self.assertFalse(load_config(parameters_path, config_path).get("runtime.dry_run.arm"))
            merged = load_config(
                parameters_path,
                config_path,
                overrides={"runtime": {"dry_run": {"arm": True}}},
            )
            self.assertTrue(merged.get("runtime.dry_run.arm"))


if __name__ == "__main__":
    unittest.main()
