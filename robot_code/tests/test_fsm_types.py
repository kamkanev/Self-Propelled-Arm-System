import unittest

from demo_core.fsm_types import MissionContext, MissionState, TargetType


class MissionContextTest(unittest.TestCase):
    def test_completed_pickups_counts_each_pickup_only_once(self):
        context = MissionContext()
        context.mark_pickup()
        context.mark_pickup()
        self.assertEqual(context.completed_pickups, 1)
        context.mark_release()
        context.mark_pickup()
        self.assertEqual(context.completed_pickups, 2)

    def test_completed_pickups_starts_at_zero(self):
        context = MissionContext()
        self.assertEqual(context.completed_pickups, 0)

    def test_vague_map_tracks_target_type(self):
        context = MissionContext()
        context.remember_target(TargetType.CAN, {"found": True})
        self.assertIn("can", context.vague_map)
        context.forget_target(TargetType.CAN)
        self.assertNotIn("can", context.vague_map)


if __name__ == "__main__":
    unittest.main()
