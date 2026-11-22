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

    def move_base_to_station(self, world, k, tuck=True):
        world.station = k
        self.log.append(("base", k) if tuck else ("base, no tuck", k))

    def run_plan(self, world, waypoints, speed, label="", held=None):
        self.log.append((label, world.station))

    def run_waypoints(self, world, waypoints, speed, label="", first_index=1):
        self.log.append((label, world.station))


class FakeWorld:
    def __init__(self, cfg, tower):
        self.cfg, self.station = cfg, 0
        self.poses = {b: r.X_WB.copy() for b, r in tower.blocks.items()}

    def X_WR(self):
        return base.station_pose(self.cfg, self.station)

    def block_poses(self):
        return {b: X.copy() for b, X in self.poses.items()}


def _pose(yaw, xyz):
    X = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    X[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    X[:3, 3] = xyz
    return X


class TestGraspCorrection(unittest.TestCase):
    """Aim the block, not the hand (D-08): pure numpy, no Drake."""

    def test_the_block_lands_on_the_target_despite_an_offset_in_the_hand(self):
        X_RT = _pose(0.3, [0.9, 0.05, 0.25])  # where the block should land
        X_RG_rel = _pose(1.2, [0.85, 0.1, 0.30])  # the plan's release hand pose for X_RT
        X_GB_actual = np.linalg.inv(X_RG_rel) @ X_RT
        X_GB_actual[:3, 3] += [0.001, 0.006, -0.0005]  # the block slid 6 mm along the reach axis
        X_RG_now = _pose(-0.7, [0.4, -0.2, 0.5])  # the hand is somewhere else now (carry pose)
        X_RT_new = mover.grasp_corrected_target(X_RT, X_RG_rel, X_RG_now, X_RG_now @ X_GB_actual)
        X_RG_rel_new = X_RG_rel.copy()
        X_RG_rel_new[:3, 3] += X_RT_new[:3, 3] - X_RT[:3, 3]  # the plan moves rigidly with its target
        np.testing.assert_allclose((X_RG_rel_new @ X_GB_actual)[:3, 3], X_RT[:3, 3], atol=1e-12)
        np.testing.assert_allclose(X_RT_new[:3, :3], X_RT[:3, :3])

    def test_no_offset_no_change(self):
        X_RT, X_RG_rel = _pose(0.3, [0.9, 0.05, 0.25]), _pose(1.2, [0.85, 0.1, 0.30])
        X_RB = X_RT.copy()
        np.testing.assert_allclose(mover.grasp_corrected_target(X_RT, X_RG_rel, X_RG_rel, X_RB), X_RT)

    def test_release_is_the_first_wider_opening(self):
        from jenga.waypoint import Waypoint
        cfg = make_cfg()
        wps = [Waypoint(None, 0.07), Waypoint(None, 0.07), Waypoint(None, 0.11), Waypoint(None, None)]
        self.assertEqual(mover.release_index(wps, cfg), 2)
        self.assertIsNone(mover.release_index(wps[:2], cfg))


@unittest.skipUnless(HAVE_DRAKE, "needs pydrake")
class TestRealMoverSteps(unittest.TestCase):
    def run_turn(self, cfg, method, block_id, place_station):
        cfg.PLACE_CORRECT_GRASP = False  # the recorded plans are not waypoints and FakeWorld has no arm
        cfg.PEDESTAL_CORRECT_GRASP = False
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
        # pedestal and regrab it there (two plans, so each starts with a joint move), place from 1; no tuck after
        # the regrab, so the end-pinched block is not swung through the ~150 deg wrist turn (O-5)
        log, info = self.run_turn(make_cfg(EXTRACTION_METHOD="push"), push, 1, place_station=1)
        self.assertEqual(log, [("base", 1), ("push", 1), ("base", 3), ("pull", 3), ("pedestal", 3),
                               ("pedestal set-down", 3), ("regrab", 3), ("base, no tuck", 1), ("place", 1)])
        self.assertEqual((info.station_pull, info.station_place), (3, 1))

    def test_end_pinch_still_runs_pull_then_place(self):
        log, info = self.run_turn(make_cfg(EXTRACTION_METHOD="end_pinch"), end_pinch, 0, place_station=1)
        self.assertEqual(log, [("base", 0), ("pull", 0), ("base, no tuck", 1), ("place", 1)])
        self.assertEqual((info.station_pull, info.station_place), (0, 1))

    def test_mixed_pushes_a_middle_block_and_end_pinches_an_edge_block(self):
        cfg = make_cfg(EXTRACTION_METHOD="mixed")
        log, _ = self.run_turn(cfg, push, 1, place_station=1)  # block 1: middle of layer 0
        self.assertEqual([step for step, _ in log if not step.startswith("base")],
                         ["push", "pull", "pedestal", "pedestal set-down", "regrab", "place"])
        log, _ = self.run_turn(cfg, end_pinch, 0, place_station=1)  # block 0: edge of layer 0
        self.assertEqual([step for step, _ in log if not step.startswith("base")], ["pull", "place"])


class BaseOnlyWorld:
    """Just enough of a Sim for executor.move_base_to_station: the base jumps to each target."""

    def __init__(self, cfg):
        self.cfg, self.station, self.mobile = cfg, 0, True
        self.q = base.base_xy_yaw(cfg, 0)

    def base_q(self):
        return self.q

    def set_base_target(self, x, y, yaw):
        self.q = (x, y, yaw)

    def advance(self, dt):
        pass

    def hold(self, seconds):
        pass


@unittest.skipUnless(HAVE_DRAKE, "needs pydrake")
class TestBaseMoveTuck(unittest.TestCase):
    def move(self, **kw):
        from jenga import executor
        world = BaseOnlyWorld(make_cfg())
        with mock.patch.object(executor, "move_arm_to_q") as arm:
            executor.move_base_to_station(world, 1, **kw)
        self.assertEqual(world.station, 1)
        return arm.call_count

    def test_tucks_by_default(self):
        self.assertEqual(self.move(), 1)

    def test_tuck_false_leaves_the_arm_alone(self):
        self.assertEqual(self.move(tuck=False), 0)


if __name__ == "__main__":
    unittest.main()
