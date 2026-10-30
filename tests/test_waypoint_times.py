"""Per-leg times in controller.waypoint_times.  Needs Drake; skipped without it."""

import unittest

try:
    from pydrake.all import RigidTransform
    from jenga.controller import timed_trajectories, waypoint_times
    HAVE_DRAKE = True
except ImportError:
    HAVE_DRAKE = False

from jenga.waypoint import Waypoint


@unittest.skipUnless(HAVE_DRAKE, "needs pydrake")
class TestWaypointTimes(unittest.TestCase):
    def setUp(self):
        self.A, self.B = RigidTransform([0, 0, 0]), RigidTransform([0.1, 0, 0])

    def test_default_is_distance_over_speed_with_a_minimum(self):
        times = waypoint_times([Waypoint(self.A), Waypoint(self.B), Waypoint(self.B, 0.05)], 0.04)
        self.assertEqual([round(t, 6) for t in times], [0.0, 2.5, 2.6])

    def test_a_waypoint_sets_the_time_of_the_leg_that_arrives_at_it(self):
        times = waypoint_times([Waypoint(self.A), Waypoint(self.B, seconds=4.0), Waypoint(self.B, 0.05, seconds=1.0)], 0.04)
        self.assertEqual([round(t, 6) for t in times], [0.0, 4.0, 5.0])

    def test_the_final_standing_stop_ignores_the_last_waypoints_time(self):
        v, _ = timed_trajectories([Waypoint(self.B, seconds=5.0)], self.A, 0.11, 0.04)
        self.assertAlmostEqual(v.end_time(), 5.1)
