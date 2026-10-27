"""Forward and inverse kinematics of the arm and gripper, in the ROBOT frame.  (Drake code: UNTESTED here.)

Uses a plant that holds only the arm and gripper, welded at the origin, so blocks never get in the
solver's way (as in the pset).  Poses are for the gripper frame G: x = finger axis, y = reach axis,
z = the remaining axis.  Adapted from the pset's inv_kin.

No collision checking: the arm model has no collision geometry (DECISIONS.md, open items).
"""

import numpy as np
from pydrake.all import InverseKinematics, RigidTransform, RotationMatrix, Solve

from utils.drake_models import explain_model_download_error


def arm_models_yaml(cfg) -> str:
    """The iiwa and the gripper (welded to link 7).  Used by both the full scene and the arm-only plant.
    Verified pattern from the previous prototype (same file names, same weld)."""
    q = cfg.ARM_CARRY_Q
    joints = "\n".join(f"        iiwa_joint_{i + 1}: [{q[i]}]" for i in range(7))
    return f"""- add_model:
    name: iiwa
    file: package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
    default_joint_positions:
{joints}
- add_model:
    name: wsg
    file: package://manipulation/hydro/schunk_wsg_50_with_tip.sdf
- add_weld:
    parent: iiwa::iiwa_link_7
    child: wsg::body
    X_PC:
        translation: [0, 0, 0.09]
        rotation: !Rpy {{ deg: [90, 0, 90] }}
"""


def arm_only_scenario_yaml(cfg) -> str:
    return (
        "directives:\n"
        + arm_models_yaml(cfg)
        + "- add_weld:\n    parent: world\n    child: iiwa::iiwa_link_0\n"
    )


class RobotKinematics:
    def __init__(self, cfg):
        from manipulation.station import LoadScenario, MakeMultibodyPlant

        self.cfg = cfg
        try:
            self.plant = MakeMultibodyPlant(LoadScenario(data=arm_only_scenario_yaml(cfg)))
        except Exception as e:  # noqa: BLE001
            explain_model_download_error(e)  # exits with a helpful message if it applies
            raise
        self.iiwa = self.plant.GetModelInstanceByName("iiwa")
        self.G = self.plant.GetBodyByName("body", self.plant.GetModelInstanceByName("wsg"))
        joints = [self.plant.GetJointByName(f"iiwa_joint_{i + 1}") for i in range(7)]
        self.lower = np.array([j.position_lower_limits()[0] for j in joints])
        self.upper = np.array([j.position_upper_limits()[0] for j in joints])

    def gripper_pose(self, q) -> RigidTransform:
        """Forward kinematics: pose of G in the robot frame for arm angles q."""
        ctx = self.plant.CreateDefaultContext()
        self.plant.SetPositions(ctx, self.iiwa, np.asarray(q, float))
        return self.plant.EvalBodyPoseInWorld(ctx, self.G)

    def joint_limit_margin(self, q) -> float:
        """Smallest distance (rad) from any arm joint to its limit."""
        q = np.asarray(q, float)
        return float(np.min(np.minimum(q - self.lower, self.upper - q)))

    def solve_ik(self, X_RG: RigidTransform, q_guess=None):
        """Arm angles that put G at X_RG (joint limits respected), or None."""
        ik = InverseKinematics(self.plant)
        G = self.G.body_frame()
        p = X_RG.translation()
        ik.AddPositionConstraint(G, [0, 0, 0], self.plant.world_frame(), p, p)
        ik.AddOrientationConstraint(G, RotationMatrix(), self.plant.world_frame(), X_RG.rotation(), 0.0)
        prog = ik.get_mutable_prog()
        ctx = self.plant.CreateDefaultContext()
        if q_guess is not None:
            self.plant.SetPositions(ctx, self.iiwa, np.asarray(q_guess, float))
        prog.SetInitialGuess(ik.q(), self.plant.GetPositions(ctx))
        result = Solve(prog)
        if not result.is_success():
            return None
        self.plant.SetPositions(ctx, result.GetSolution(ik.q()))  # as in the pset: read the arm's part back
        return np.asarray(self.plant.GetPositions(ctx, self.iiwa), float)

    def solve_ik_best(self, X_RG: RigidTransform, q_ref, tries: int = 30, seed: int = 0, good_margin: float = 0.15):
        """A joint-limit-safe IK solution for X_RG that is also close to q_ref, or None.

        Tries q_ref and `tries` random starting guesses.  Among solutions with at least `good_margin` rad
        (about 9 degrees) to every joint limit it returns the one nearest q_ref, so a joint-space move
        from q_ref does not swing the arm to a different configuration; if none is that safe it returns
        the one with the largest margin.  (If a pose fails, try the hand rolled 180 degrees about G y:
        the two fingers are identical, so it is the same grasp with a different wrist.)"""
        rng = np.random.default_rng(seed)
        q_ref = np.asarray(q_ref, float)
        guesses = [q_ref] + [rng.uniform(self.lower, self.upper) for _ in range(tries)]
        sols = []
        for g in guesses:
            q = self.solve_ik(X_RG, g)
            if q is not None:
                sols.append((q, self.joint_limit_margin(q)))
        if not sols:
            return None
        good = [s for s in sols if s[1] >= good_margin]
        best = min(good, key=lambda s: np.linalg.norm(s[0] - q_ref)) if good else max(sols, key=lambda s: s[1])
        return best[0]
