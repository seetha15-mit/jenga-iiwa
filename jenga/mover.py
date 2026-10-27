"""Movers: what actually carries out one turn (pull a block, place it on top).

FakeMover  teleports the block to its target slot.  No arm, no physics for the move.  Exists only so the
           harness, logging, collapse detector and replay can be tested before the team's steps exist.
           Everything it produces is tagged mover = "fake" and goes to runs_fake/.  RESULTS ARE INVALID.
RealMover  runs the team's steps: rolls the base to a station, executes the method's plan_pull, rolls to a
           placing station if needed, and executes plan_place.

Both take a `world` (jenga.scene.Sim, or a stand-in with the same few methods in the tests).
"""

from dataclasses import dataclass

from jenga import base, clearance, executor_api  # noqa: F401  (executor_api: lazy Drake import point)
from jenga import methods


@dataclass
class TurnInfo:
    station_pull: int = -1
    station_place: int = -1


class FakeMover:
    TAG = "fake"

    def banner(self) -> str:
        bar = "*" * 78
        return (f"{bar}\n*** FAKE MOVER: blocks are TELEPORTED.  Results are INVALID and are written to "
                f"the fake-runs folder. ***\n{bar}")

    def do_turn(self, world, tower, block_id, target, cfg) -> TurnInfo:
        layer, slot = target
        world.teleport_block(block_id, tower.target_pose(layer, slot))
        world.hold(1.0)  # let the tower settle so a disturbed neighbour shows up in the poses
        return TurnInfo()


class RealMover:
    TAG = "real"

    def banner(self) -> str:
        return ""

    def do_turn(self, world, tower, block_id, target, cfg) -> TurnInfo:
        from pydrake.all import RigidTransform  # Drake only needed on the real path

        ex = executor_api.get()
        method = methods.get_method(cfg.EXTRACTION_METHOD)
        layer, slot = target

        # ---- pull
        k_pull = method.choose_station(tower, block_id, cfg, world.station)
        ex.move_base_to_station(world, k_pull)
        X_RB = RigidTransform(base.to_robot_frame(world.X_WR(), tower.block_pose(block_id)))
        ex.run_plan(world, method.plan_pull(X_RB, cfg), cfg.WAYPOINT_SPEED_MS, label="pull")

        # ---- place (the block stays in the hand while the base rolls)
        k_place = method.choose_place_station(tower, layer, slot, cfg, world.station)
        ex.move_base_to_station(world, k_place)
        X_RT = RigidTransform(base.to_robot_frame(world.X_WR(), tower.target_pose(layer, slot)))
        ex.run_plan(world, method.plan_place(X_RT, cfg), cfg.WAYPOINT_SPEED_MS, label="place")
        return TurnInfo(k_pull, k_place)


def get_mover(name: str):
    if name == "fake":
        return FakeMover()
    if name == "real":
        return RealMover()
    raise ValueError(f"MOVER must be 'fake' or 'real', got {name!r}")
