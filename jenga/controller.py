"""Moving the hand through waypoints, with ONE persistent simulator.  (Drake code: UNTESTED here.)

Adapted from our own pick_and_place.py pset solutions: pose_distance, pose_delta, waypoint_times,
timed_trajectories and JointVelocity are the pset's feedforward differential-IK controller
(gripper velocity -> pseudo-inverse Jacobian -> joint velocities -> Integrator -> iiwa.position).
Cite the course for it in the report.

The plant itself is built by hand in the pset style (jenga/scene.py), with PD-controlled joints.

What is new here: the pset builds a fresh diagram per motion.  We keep one diagram for the whole trial, so
the trajectory sits in a small system (MotionPlayer) whose python-side state is swapped BETWEEN calls to
AdvanceTo.  That is the part that has never been run; check 'executor' in run_checks.py.

Everything here works in the ROBOT frame R (arm base at the origin): the Jacobian comes from an arm-only
plant, so hand velocities are expressed in R no matter where the rolling base stands.

Limits worth knowing: the pseudo-inverse ignores joint limits (F-05 in DECISIONS.md); the executor warns when a
joint ends near one.  Swapping the controller later (e.g. impedance control): replace what feeds the
'qdot_cartesian' input in scene.py; nothing else changes.
"""

import numpy as np
from pydrake.all import (
    JacobianWrtVariable,
    LeafSystem,
    PiecewisePolynomial,
    PiecewisePose,
    RigidTransform,
)

from jenga.waypoint import Waypoint
from utils.dtsystems import DTSystem

GRIPPER_RADIUS = 0.1  # pset: weights rotation against translation when measuring pose distance


def as_transform(X) -> RigidTransform:
    """Accept a RigidTransform or a 4x4 numpy array."""
    return X if isinstance(X, RigidTransform) else RigidTransform(np.asarray(X, float))


# ---------------------------------------------------------------- trajectory helpers (from pick_and_place.py)
def pose_distance(error: np.ndarray) -> float:
    scaled = np.concatenate([GRIPPER_RADIUS * error[:3], error[3:]])
    return float(np.linalg.norm(scaled))


def pose_delta(X_RG: RigidTransform, X_RG_target: RigidTransform) -> np.ndarray:
    """[rotation vector (3); translation (3)] from X_RG to X_RG_target."""
    R_error = X_RG_target.rotation() @ X_RG.rotation().inverse()
    angle_axis = R_error.ToAngleAxis()
    return np.concatenate([angle_axis.angle() * angle_axis.axis(), X_RG_target.translation() - X_RG.translation()])


def waypoint_times(stops, speed: float):
    """When to be at each stop.  A stop with its own `seconds` takes that long to reach; otherwise travel
    time = distance / speed, at least 0.1 s (closing the fingers is a zero-distance move but time must still grow)."""
    times = [0.0]
    for i in range(len(stops) - 1):
        leg = stops[i + 1].seconds
        if leg is None:
            distance = pose_distance(pose_delta(stops[i].X_RG, stops[i + 1].X_RG))
            leg = max(distance / speed, 0.1)
        times.append(times[i] + leg)
    return times


def timed_trajectories(waypoints, X_RG_start, start_opening: float, speed: float):
    """(hand velocity trajectory, finger opening trajectory) for a plan.  Adds a stop at the starting pose (the
    controller is feedforward) and repeats the last stop so the trajectory ends standing still."""
    stops = [Waypoint(X_RG_start, start_opening)] + list(waypoints) + [waypoints[-1]._replace(seconds=None)]
    times = waypoint_times(stops, speed)
    poses = [w.X_RG for w in stops]
    openings = [w.opening for w in stops]
    pose_trajectory = PiecewisePose.MakeLinear(times, poses)
    velocity_trajectory = pose_trajectory.MakeDerivative()
    fingers = PiecewisePolynomial.ZeroOrderHold(times, np.array(openings, dtype=float).reshape(1, -1))
    return velocity_trajectory, fingers


def JointVelocity(arm_plant):
    """System: hand velocity V_RG (6) and arm angles (7) in; joint velocities (7) out.  `arm_plant` holds
    only the arm and gripper, welded at the origin (jenga.kinematics.RobotKinematics.plant)."""
    plant_context = arm_plant.CreateDefaultContext()
    iiwa = arm_plant.GetModelInstanceByName("iiwa")
    G = arm_plant.GetBodyByName("body", arm_plant.GetModelInstanceByName("wsg")).body_frame()
    W = arm_plant.world_frame()
    cols = slice(
        arm_plant.GetJointByName("iiwa_joint_1").velocity_start(),
        arm_plant.GetJointByName("iiwa_joint_7").velocity_start() + 1,
    )

    def solve(_state, inputs):
        V_RG, q = inputs
        arm_plant.SetPositions(plant_context, iiwa, q)
        J_G = arm_plant.CalcJacobianSpatialVelocity(plant_context, JacobianWrtVariable.kV, G, [0, 0, 0], W, W)
        return np.linalg.pinv(J_G[:, cols]) @ V_RG

    return DTSystem(
        [6, 7], 0, 7, None, solve, output_depends_on_input=True,
        input_port_name=["V_RG", "iiwa.position"], output_port_name="iiwa.velocity", name="JointVelocity",
    )


# ---------------------------------------------------------------- systems whose python state changes between AdvanceTo calls
class MotionPlayer(LeafSystem):
    """Plays whatever motion is loaded, and outputs zero velocity when idle.

    Outputs: V_RG (6, hand velocity for the Cartesian controller), qdot_joint (7, for joint-space moves),
    opening (1, finger command).  Load a motion with start_cartesian / start_joint, then call
    simulator.AdvanceTo; call stop() afterwards.  The loaded motion is plain python state: it is only
    safe to change it between AdvanceTo calls."""

    def __init__(self, idle_opening: float):
        super().__init__()
        self.mode = "idle"  # "idle", "cartesian" or "joint"
        self.last_opening = float(idle_opening)
        self._t0 = 0.0
        self._v = None
        self._f = None
        self._dq = None
        self._T = 0.0
        self.DeclareVectorOutputPort("V_RG", 6, self._calc_v)
        self.DeclareVectorOutputPort("qdot_joint", 7, self._calc_qdot)
        self.DeclareVectorOutputPort("opening", 1, self._calc_opening)

    # -- loading
    def start_cartesian(self, v_traj, finger_traj, t0: float):
        self.mode, self._v, self._f, self._t0 = "cartesian", v_traj, finger_traj, t0
        self.last_opening = float(finger_traj.value(finger_traj.end_time())[0, 0])

    def start_joint(self, dq, T: float, opening: float, t0: float):
        """Move the arm by dq (7) in T seconds with a smooth bell-shaped velocity profile."""
        self.mode, self._dq, self._T, self._t0 = "joint", np.asarray(dq, float), float(T), t0
        self.last_opening = float(opening)

    def stop(self, opening=None):
        self.mode = "idle"
        if opening is not None:
            self.last_opening = float(opening)

    # -- outputs
    def _calc_v(self, context, output):
        t = context.get_time() - self._t0
        if self.mode == "cartesian" and 0.0 <= t <= self._v.end_time():
            output.SetFromVector(np.asarray(self._v.value(t)).reshape(6))
        else:
            output.SetFromVector(np.zeros(6))

    def _calc_qdot(self, context, output):
        t = context.get_time() - self._t0
        if self.mode == "joint" and 0.0 <= t <= self._T:
            output.SetFromVector(self._dq / self._T * (1.0 - np.cos(2 * np.pi * t / self._T)))
        else:
            output.SetFromVector(np.zeros(7))

    def _calc_opening(self, context, output):
        if self.mode == "cartesian":
            t = min(max(context.get_time() - self._t0, 0.0), self._f.end_time())
            output.SetFromVector([float(self._f.value(t)[0, 0])])
        else:
            output.SetFromVector([self.last_opening])


class VelocitySelector(LeafSystem):
    """Passes on the Cartesian controller's joint velocities, or the joint-space move's, depending on the
    MotionPlayer's mode."""

    def __init__(self, player: MotionPlayer):
        super().__init__()
        self._player = player
        self._cart = self.DeclareVectorInputPort("qdot_cartesian", 7)
        self._joint = self.DeclareVectorInputPort("qdot_joint", 7)
        self.DeclareVectorOutputPort("qdot", 7, self._calc)

    def _calc(self, context, output):
        port = self._joint if self._player.mode == "joint" else self._cart
        output.SetFromVector(port.Eval(context))


class FirstEntries(LeafSystem):
    """Passes on the first n_out entries of its input (for example the joint positions q out of the [q, v] state)."""

    def __init__(self, n_in: int, n_out: int):
        super().__init__()
        self._n = n_out
        self._u = self.DeclareVectorInputPort("in", n_in)
        self.DeclareVectorOutputPort("out", n_out, self._calc)

    def _calc(self, context, output):
        output.SetFromVector(np.asarray(self._u.Eval(context))[: self._n])


class PositionToState(LeafSystem):
    """Turns a position command q (n) into the desired STATE [q, 0] (2n) that a PD-controlled model instance
    takes at its desired-state port (zero desired velocity)."""

    def __init__(self, n: int):
        super().__init__()
        self._n = n
        self._u = self.DeclareVectorInputPort("position", n)
        self.DeclareVectorOutputPort("desired_state", 2 * n, self._calc)

    def _calc(self, context, output):
        output.SetFromVector(np.concatenate([np.asarray(self._u.Eval(context)), np.zeros(self._n)]))


class OpeningToFingers(LeafSystem):
    """Finger opening command (m) -> the two finger joint positions [-opening/2, +opening/2], as in the psets."""

    def __init__(self):
        super().__init__()
        self._u = self.DeclareVectorInputPort("opening", 1)
        self.DeclareVectorOutputPort("finger_positions", 2, self._calc)

    def _calc(self, context, output):
        o = float(np.asarray(self._u.Eval(context))[0])
        output.SetFromVector([-o / 2, o / 2])


class BaseCommand(LeafSystem):
    """Holds the rolling base's commanded pose (x, y, yaw) and outputs it as a desired state [x, y, yaw, 0, 0, 0].
    Set with .set(x, y, yaw) between AdvanceTo calls."""

    def __init__(self, initial_xy_yaw):
        super().__init__()
        self._v = np.zeros(6)
        self.set(*initial_xy_yaw)
        self.DeclareVectorOutputPort("desired_state", 6, self._calc)

    def set(self, x, y, yaw):
        self._v[:3] = [x, y, yaw]

    def _calc(self, context, output):
        output.SetFromVector(self._v)
