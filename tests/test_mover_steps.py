"""RealMover runs each method's steps in the right order, at the right stations.  No physics: the executor
and the team's plans are replaced by recorders.  Needs Drake (RealMover builds RigidTransforms)."""

import unittest
from unittest import mock

import numpy as np

try:
    import pydrake  # noqa: F401
    HAVE_DRAKE = True
except ImportError:
    HAVE_DRAKE = False

from jenga import base, executor_api, mover
from jenga.methods import end_pinch, push
from jenga.tower import TowerState
from tests.helpers import make_cfg


class FakeExecutor:
    """Records base moves and plans instead of simulating them."""

    def __init__(self):
        self.log = []

    def move_base_to_station(self, world, k):
        world.station = k
        self.log.append(("base", k))

    def run_plan(self, world, waypoints, speed, label=""):
        self.log.append((label, world.station))


class FakeWorld:
    def __init__(self, cfg, tower):
        self.cfg, self.station = cfg, 0
        self.poses = {b: r.X_WB.copy() for b, r in tower.blocks.items()}

    def X_WR(self):
        return base.station_pose(self.cfg, self.station)

    def block_poses(self):
        return {b: X.copy() for b, X in self.poses.items()}


@unittest.skipUnless(HAVE_DRAKE, "needs pydrake")
class TestRealMoverSteps(unittest.TestCase):
    def run_turn(self, cfg, method, block_id, place_station):
        tower = TowerState(cfg)
        world, ex = FakeWorld(cfg, tower), FakeExecutor()
        plans = {name: mock.patch.object(method, name, return_value=["wp"])
                 for name in ("plan_push", "plan_pull", "plan_ped_place", "plan_regrab", "plan_place")
                 if hasattr(method, name)}
        with mock.patch.object(executor_api, "get", return_value=ex), \
                mock.patch.object(method, "choose_place_station", return_value=place_station):
            for p in plans.values():
                p.start()
            try:
                info = mover.RealMover().do_turn(world, tower, block_id, (3, 0), cfg)
            finally:
                for p in plans.values():
                    p.stop()
        return ex.log, info

    def test_push_runs_push_pull_pedestal_regrab_place_at_their_stations(self):
        # block 1 (layer 0, middle) from station 0: push from 1 (nearest of 1 and 3), pull from 3, set it on the
        # pedestal and regrab it there (two plans, so each starts with a joint move), place from 1
        log, info = self.run_turn(make_cfg(EXTRACTION_METHOD="push"), push, 1, place_station=1)
        self.assertEqual(log, [("base", 1), ("push", 1), ("base", 3), ("pull", 3), ("pedestal", 3), ("regrab", 3),
                               ("base", 1), ("place", 1)])
        self.assertEqual((info.station_pull, info.station_place), (3, 1))

    def test_end_pinch_still_runs_pull_then_place(self):
        log, info = self.run_turn(make_cfg(EXTRACTION_METHOD="end_pinch"), end_pinch, 0, place_station=1)
        self.assertEqual(log, [("base", 0), ("pull", 0), ("base", 1), ("place", 1)])
        self.assertEqual((info.station_pull, info.station_place), (0, 1))


if __name__ == "__main__":
    unittest.main()
