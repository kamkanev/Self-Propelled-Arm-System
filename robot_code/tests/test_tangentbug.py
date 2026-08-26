import unittest

import numpy as np

from demo_core.tangentbug import DepthTangentBugPlanner


SETTINGS = {
    "obstacle_depth": 1.2,
    "clear_depth": 1.6,
    "profile_columns": 40,
    "profile_row_start": 0.4,
    "profile_row_end": 0.9,
    "smoothing_window": 3,
    "minimum_gap_columns": 4,
}


class TangentBugTest(unittest.TestCase):
    def test_clear_center_returns_path_clear(self):
        depth = np.full((60, 80), 2.0, dtype=np.float32)
        plan = DepthTangentBugPlanner(SETTINGS).plan(depth)
        self.assertTrue(plan.path_clear)
        self.assertEqual(plan.action, "forward")

    def test_center_obstacle_selects_side_candidate(self):
        depth = np.full((60, 80), 2.0, dtype=np.float32)
        depth[25:55, 30:50] = 0.8
        plan = DepthTangentBugPlanner(SETTINGS).plan(depth)
        self.assertFalse(plan.path_clear)
        self.assertIn(plan.action, ("left", "right", "forward"))
        self.assertTrue(plan.candidates)

    def test_debug_overlay_keeps_frame_shape(self):
        depth = np.full((60, 80), 2.0, dtype=np.float32)
        planner = DepthTangentBugPlanner(SETTINGS)
        plan = planner.plan(depth)
        frame = np.zeros((60, 80, 3), dtype=np.uint8)
        self.assertEqual(planner.draw_debug(frame, plan).shape, frame.shape)


if __name__ == "__main__":
    unittest.main()
