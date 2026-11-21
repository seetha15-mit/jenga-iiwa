"""Swing clearance (jenga/clearance.py): geometry, the order IK solutions are tried in, the choice, and that the
executor is unchanged with the check off.  Pure Python except the last class (needs Drake)."""

import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from jenga import clearance
from jenga.errors import MotionFailed
from tests.helpers import make_cfg


class TestGeometry(unittest.TestCase):
    def test_surface_points_lie_on_the_surface_and_are_close_together(self):
        dims = (0.1, 0.04, 0.02)
        P = clearance.surface_points(dims, (1.0, 0.0, 0.0), step=0.01)
        d = np.abs(P - [1.0, 0, 0]) - np.array(dims) / 2
        self.assertTrue(np.all(d <= 1e-12))
        self.assertTrue(np.all(np.isclose(d.max(axis=1), 0.0)))  # every point is on a face
        self.assertTrue(np.any(np.all(np.isclose(P, [1.0 - 0.05, -0.02, -0.01]), axis=1)))  # corners included

    def test_box_distance_outside_inside_and_turned(self):
        X = np.eye(4)
        half = (0.05, 0.02, 0.01)
        np.testing.assert_allclose(clearance.box_distance([[0.15, 0, 0], [0, 0, 0], [0.08, 0.06, 0]], X, half),
                                   [0.10, -0.01, 0.05])
        X = clearance._pose((0, 0, 0), np.pi / 2)  # long side now along world y
        np.testing.assert_allclose(clearance.box_distance([[0, 0.15, 0]], X, half), [0.10])

    def test_joint_path_leaves_out_the_ends_and_respects_the_step(self):
        path = clearance.joint_path(np.zeros(7), np.r_[1.0, np.zeros(6)], 0.1)
        self.assertEqual(len(path), 9)
        self.assertTrue(np.all(np.diff(path[:, 0]) <= 0.1 + 1e-12))
        self.assertGreater(path[0, 0], 0.0)
        self.assertLess(path[-1, 0], 1.0)

    def test_obstacles_skip_the_held_block(self):
        cfg = make_cfg()
        names = [n for n, _, _ in clearance.obstacles(cfg, {1: np.eye(4), 2: np.eye(4)}, skip=(1,))]
        self.assertIn("table", names)
        self.assertIn("block_2", names)
        self.assertNotIn("block_1", names)
        self.assertEqual(sum(n.startswith("pedestal") for n in names), 4)

    def test_preference_order_starts_with_solve_ik_best_choice(self):
        margin = lambda q: q[1]  # noqa: E731
        sols = [np.array([3.0, 0.2]), np.array([1.0, 0.2]), np.array([0.5, 0.1]), np.array([0.2, 0.05])]
        order = clearance.preference_order(sols, np.zeros(2), margin, 0.15)
        self.assertEqual([q[0] for q in order], [1.0, 3.0, 0.5, 0.2])  # safe ones nearest first, then by margin


def _fake_world(cfg, solutions, margin, held_pose=None):
    """The hand's position is q[:3]; q[3] going from 0 to 1 (or back) dips the hand 1 m on the way (through the
    table), so two solutions with the same end pose can have different paths.  The hand starts 0.5 m up."""
    def fk(q):
        X = np.eye(4)
        X[:3, 3] = [q[0], q[1], q[2] - np.sin(np.pi * q[3])]
        return SimpleNamespace(GetAsMatrix4=lambda X=X: X)

    kin = SimpleNamespace(gripper_pose=fk, ik_solutions=lambda X, q: solutions, joint_limit_margin=margin)
    poses = {} if held_pose is None else {5: held_pose}
    return SimpleNamespace(cfg=cfg, kin=kin, arm_q=lambda: np.array([0, 0, 0.5, 0, 0, 0, 0.0]),
                           X_WR=lambda: np.eye(4), block_poses=lambda: dict(poses))


class TestChooseSwing(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg(CHECK_SWING_CLEARANCE=True)
        self.dips = np.array([0.4, 0, 0.5, 1.0, 0, 0, 0])  # preferred (safe margin, see margin below) but dips
        self.flat = np.array([0.4, 0, 0.5, 0.0, 1.0, 0, 0])  # same end pose, straight path, smaller margin
        self.margin = lambda q: 0.5 if q[4] == 0 else 0.1  # noqa: E731

    def test_takes_the_first_clear_solution(self):
        w = _fake_world(self.cfg, [self.dips, self.flat], self.margin)
        q, info = clearance.choose_swing(w, None, 0.11)
        np.testing.assert_array_equal(q, self.flat)
        self.assertEqual((info["swing_tried"], info["swing_options"]), (2, 2))
        self.assertGreaterEqual(info["swing_clear_mm"], self.cfg.SWING_CLEARANCE_M * 1000)

    def test_takes_the_usual_choice_when_it_is_clear(self):
        w = _fake_world(self.cfg, [self.flat], self.margin)
        q, info = clearance.choose_swing(w, None, 0.11)
        self.assertEqual(info["swing_tried"], 1)

    def test_fails_when_none_is_clear(self):
        w = _fake_world(self.cfg, [self.dips], self.margin)
        with self.assertRaises(MotionFailed) as e:
            clearance.choose_swing(w, None, 0.11, label="pedestal")
        self.assertIn("none of 1 IK solutions", str(e.exception))  # the dip hits the platform first, then the table

    def test_fails_without_ik(self):
        with self.assertRaises(MotionFailed):
            clearance.choose_swing(_fake_world(self.cfg, [], self.margin), None, 0.11)

    def test_the_held_block_moves_with_the_hand_and_is_not_an_obstacle(self):
        X = np.eye(4)
        X[:3, 3] = [0, 0.10, 0.5]  # in the hand: between the fingers (pads at x = +-57.5 mm), y 82-118 mm
        w = _fake_world(self.cfg, [self.flat], self.margin, held_pose=X)
        q, _ = clearance.choose_swing(w, None, 0.11, held=5)
        np.testing.assert_array_equal(q, self.flat)
        # the same block NOT held is an obstacle the hand starts inside: no swing is clear
        with self.assertRaises(MotionFailed):
            clearance.choose_swing(w, None, 0.11)


try:
    from jenga import executor
    HAVE_DRAKE = True
except ImportError:
    HAVE_DRAKE = False


@unittest.skipUnless(HAVE_DRAKE, "needs pydrake")
class TestExecutorSwitch(unittest.TestCase):
    def _world(self, check):
        best = mock.Mock(return_value=np.zeros(7))
        return SimpleNamespace(cfg=make_cfg(CHECK_SWING_CLEARANCE=check), arm_q=lambda: np.ones(7),
                               kin=SimpleNamespace(solve_ik_best=best), player=SimpleNamespace(last_opening=0.11)), best

    def test_off_uses_solve_ik_best_exactly_as_before(self):
        from pydrake.all import RigidTransform
        w, best = self._world(False)
        with mock.patch.object(executor, "move_arm_to_q") as move, mock.patch.object(executor.clearance, "choose_swing") as ch:
            executor.move_arm_to_pose(w, RigidTransform(), 0.05, label="x", held=3)
        best.assert_called_once()
        ch.assert_not_called()
        move.assert_called_once()

    def test_on_uses_the_clearance_choice(self):
        from pydrake.all import RigidTransform
        w, best = self._world(True)
        with mock.patch.object(executor, "move_arm_to_q") as move, \
                mock.patch.object(executor.clearance, "choose_swing", return_value=(np.full(7, 2.0), {"swing_tried": 1})) as ch:
            executor.move_arm_to_pose(w, RigidTransform(), 0.05, label="x", held=3)
        best.assert_not_called()
        self.assertEqual(ch.call_args.args[2:4], (0.11, 3))  # the wider of the current and new opening; the held block
        np.testing.assert_array_equal(move.call_args.args[1], np.full(7, 2.0))
        self.assertEqual(w.last_swing, {"swing_tried": 1})


if __name__ == "__main__":
    unittest.main()
