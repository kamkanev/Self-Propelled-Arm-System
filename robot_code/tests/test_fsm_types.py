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

    def test_visual_target_memory_does_not_create_map_entries(self):
        context = MissionContext()
        context.remember_target(TargetType.CAN, {"found": True})
        self.assertIsNone(context.vague_map)
        self.assertTrue(context.last_observation["found"])
        context.forget_target(TargetType.CAN)
        self.assertIsNone(context.last_observation)


if __name__ == "__main__":
    unittest.main()
