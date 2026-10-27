"""Test helpers: a stand-in for the Drake simulator, and movers for exercising the harness."""

from types import SimpleNamespace

import numpy as np

import config as _config
from jenga.mover import TurnInfo


def make_cfg(**over):
    d = {k: getattr(_config, k) for k in dir(_config) if k.isupper()}
    d.update(over)
    return SimpleNamespace(**d)


class MockWorld:
    """Same few methods as jenga.scene.Sim that the harness and movers use, with no physics."""

    def __init__(self, cfg, tower, friction, record_replay):
        self._t = 0.0
        self.station = 0
        self.poses = {b: r.X_WB.copy() for b, r in tower.blocks.items()}
        self._friction = friction  # hidden truth; strategies never get this object
        self.events = []  # ("hold", seconds) and ("teleport", block, time) in order

    def time(self):
        return self._t

    def block_poses(self):
        return {b: X.copy() for b, X in self.poses.items()}

    def teleport_block(self, bid, X):
        self.events.append(("teleport", bid, self._t))
        self.poses[bid] = np.array(X, float)

    def hold(self, seconds):
        self.events.append(("hold", seconds))
        self._t += seconds

    def save_replay(self, path):
        return False

    def save_force_log(self, path):
        return False


class RealLikeMover:
    """Behaves like the fake mover but claims to be real (for testing results handling only)."""
    TAG = "real"

    def banner(self):
        return ""

    def do_turn(self, world, tower, block_id, target, cfg):
        world.teleport_block(block_id, tower.target_pose(*target))
        world.hold(1.0)
        return TurnInfo()


class DoNothingMover(RealLikeMover):
    def do_turn(self, world, tower, block_id, target, cfg):
        world.hold(1.0)
        return TurnInfo()


class DisturbMover(RealLikeMover):
    """Moves the block to its target AND shoves another block 5 cm."""

    def do_turn(self, world, tower, block_id, target, cfg):
        other = next(b for b in tower.blocks if b != block_id)
        X = world.poses[other].copy()
        X[0, 3] += 0.05
        world.poses[other] = X
        return super().do_turn(world, tower, block_id, target, cfg)


class FailingMover(RealLikeMover):
    def do_turn(self, world, tower, block_id, target, cfg):
        from jenga.errors import MotionFailed

        raise MotionFailed("no IK solution (test)")
