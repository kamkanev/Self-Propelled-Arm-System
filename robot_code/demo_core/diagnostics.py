from __future__ import print_function

from .state_machine import MissionServices


class DemoDiagnostics(object):
    """Shared camera and detector lifecycle for board-side diagnostics."""

    def __init__(self, config, services=None):
        self.config = config
        self.services = services or MissionServices.from_config(config)

    def start_camera(self):
        if self.services.depth.depth is None:
            self.services.depth.start()
        return self.read_frame()

    def read_frame(self):
        return self.services.depth.read_frame()

    def release_camera(self):
        self.services.depth.stop()
        return True

    def reset_models(self):
        self.services.can_detector.reset()
        self.services.bin_detector.detector = None
        self.services.bin_detector.aruco_dict = None
        return True

    def load_can(self, reload_model=False):
        if reload_model:
            self.services.can_detector.reset()
        self.services.can_detector.load()
        return True

    def load_tag(self, reload_model=False):
        if reload_model:
            self.services.bin_detector.detector = None
            self.services.bin_detector.aruco_dict = None
        self.services.bin_detector.load()
        return True

    def load_all(self, reload_models=False):
        self.load_can(reload_models)
        self.load_tag(reload_models)
        return True

    def observe_can(self):
        return self.services.can_detector.detect(self.read_frame())

    def observe_tag(self):
        return self.services.bin_detector.detect(self.read_frame())

    def observe_depth(self):
        frame = self.read_frame()
        return {
            "lens": self.services.depth.observe_lens_center_frame(frame),
            "obstacle": self.services.depth.observe_frame("obstacle_depth_roi", frame),
        }

    def preflight(self, reload_models=False):
        self.start_camera()
        before = self.observe_depth()
        self.load_all(reload_models)
        return {
            "depth_before_models": before,
            "depth_after_models": self.observe_depth(),
            "can": self.observe_can(),
            "tag": self.observe_tag(),
        }
