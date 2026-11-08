"""Movers: what actually carries out one turn (pull a block, place it on top).

FakeMover  teleports the block to its target slot.  No arm, no physics for the move.  Exists only so the
           harness, logging, collapse detector and replay can be tested before the team's steps exist.
           Everything it produces is tagged mover = "fake" and goes to runs_fake/.  RESULTS ARE INVALID.
RealMover  runs the team's steps: rolls the base to a station, executes the method's plan_pull, rolls to a
           placing station if needed, and executes plan_place.  A method with a plan_push (the push method)
           instead runs push, pull from the opposite station, regrab, then place.

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

        def in_R(X_WB):  # a world pose in the robot frame, where the base stands NOW
            return RigidTransform(base.to_robot_frame(world.X_WR(), X_WB))

        if hasattr(method, "plan_push"):
            # ---- push (push method): from one end, the block slides part way out the far side
            ex.move_base_to_station(world, method.choose_station(tower, block_id, cfg, world.station))
            ex.run_plan(world, method.plan_push(in_R(tower.block_pose(block_id)), cfg), cfg.WAYPOINT_SPEED_MS,
                        label="push")
            # ---- pull from the far end, then regrab; the block has moved, so its pose is read again
            k_pull = method.choose_pull_station(tower, block_id, cfg, world.station)
            ex.move_base_to_station(world, k_pull)
            ex.run_plan(world, method.plan_pull(in_R(world.block_poses()[block_id]), cfg), cfg.WAYPOINT_SPEED_MS,
                        label="pull")
            # ---- set it on a pedestal, then regrab it from where it actually landed (D-22); each is its own plan,
            # so each starts with a joint move (this clears the hand's drift before the regrab grasp)
            ex.run_plan(world, method.plan_ped_place(in_R(world.block_poses()[block_id]), cfg), cfg.WAYPOINT_SPEED_MS,
                        label="pedestal")
            ex.run_plan(world, method.plan_regrab(in_R(world.block_poses()[block_id]), cfg), cfg.WAYPOINT_SPEED_MS,
                        label="regrab")
        else:
            # ---- pull
            k_pull = method.choose_station(tower, block_id, cfg, world.station)
            ex.move_base_to_station(world, k_pull)
            ex.run_plan(world, method.plan_pull(in_R(tower.block_pose(block_id)), cfg), cfg.WAYPOINT_SPEED_MS,
                        label="pull")

        # ---- place (the block stays in the hand while the base rolls)
        k_place = method.choose_place_station(tower, layer, slot, cfg, world.station)
        ex.move_base_to_station(world, k_place)
        ex.run_plan(world, method.plan_place(in_R(tower.target_pose(layer, slot)), cfg), cfg.WAYPOINT_SPEED_MS,
                    label="place")
        return TurnInfo(k_pull, k_place)


def get_mover(name: str):
    if name == "fake":
        return FakeMover()
    if name == "real":
        return RealMover()
    raise ValueError(f"MOVER must be 'fake' or 'real', got {name!r}")
