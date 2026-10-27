"""The Drake world and ONE persistent simulator per trial.  (Drake code: UNTESTED here.)

BUILT BY HAND, pset style (like iiwa_1.py and mobile_base_ik.py): AddMultibodyPlantSceneGraph + Parser, no hardware
station.  Why: the station's iiwa driver aborts when the arm is welded to a body that has joints of its own
(measured with diagnose_base.py: Drake stops with `inverse_dynamics_controller.cc: num_positions == dim`).

What is in the world: the table (top at z = 0), the platform, the tower's blocks (one free body each), and the
robot (iiwa + WSG gripper) on a rolling base made of three joints (x, y, yaw).  The tower never moves between
turns; the base rolls between stations instead (jenga/base.py).

How the joints are driven: every actuated joint (arm, fingers, base) is a PD-controlled joint of the plant itself
(MultibodyPlant's own PD actuators: stiffness and damping per joint, handled implicitly by the solver), and the plant
takes a desired state for each model instance.  Gravity is switched off for the ARM only (an ideal
gravity-compensated arm, as the manipulation station does); the gripper keeps its weight, so the wrist force
reading includes the hand's weight as in our measurements.

`Sim` is the `world` that the executor, movers and harness talk to.  Its methods:
    time(), advance(s), hold(s)          move simulated time forward
    block_poses()                        TRUE 4x4 poses of all blocks (evaluation and tower bookkeeping)
    teleport_block(id, X)                fake mover only
    arm_q(), base_q(), X_WR()            robot state; X_WR = robot base frame in the world
    start_cartesian / start_joint / ...  used by the executor
    wrist_force_raw()                    raw reading in the gripper frame (see jenga/sensing.py)
    save_replay(path), save_force_log(path)
"""

import csv
from pathlib import Path

import numpy as np
from manipulation import ConfigureParser
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ContactModel,
    DiagramBuilder,
    DiscreteContactApproximation,
    Integrator,
    Meshcat,
    MeshcatVisualizer,
    Parser,
    PdControllerGains,
    RigidTransform,
    RollPitchYaw,
    Simulator,
)

from jenga import base as base_mod
from jenga import geometry as geo
from jenga.controller import (
    BaseCommand,
    FirstEntries,
    JointVelocity,
    MotionPlayer,
    OpeningToFingers,
    PositionToState,
    VelocitySelector,
)
from jenga.kinematics import RobotKinematics
from utils.drake_models import explain_model_download_error

BASE_JOINTS = ("base_x", "base_y", "base_yaw")
BOX_BODY = "base_link"  # name of the single body in every box SDF
IIWA_URL = "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf"
WSG_URL = "package://manipulation/hydro/schunk_wsg_50_with_tip.sdf"
IIWA_JOINTS = tuple(f"iiwa_joint_{i}" for i in range(1, 8))
WSG_JOINTS = ("left_finger_sliding_joint", "right_finger_sliding_joint")

_MESHCAT = None  # one server per process, only started when replays are recorded


def _recording_meshcat():
    global _MESHCAT
    if _MESHCAT is None:
        _MESHCAT = Meshcat()  # no browser needed: we only record and save the replay
    return _MESHCAT


# ---------------------------------------------------------------- SDF files (adapted from the pset's write_box_sdf)
def write_box_sdf(path: Path, name, size, rgb, mass, mu, cfg) -> Path:
    """A coloured box.  Body frame at the centre of the BOTTOM face.  Friction mu on all faces.
    Hydroelastic contact follows config.CONTACT_MODEL (under plain point contact two boxes touch only at corners)."""
    sx, sy, sz = size
    r, g, b = rgb
    ixx, iyy, izz = (mass * (sy**2 + sz**2) / 12, mass * (sx**2 + sz**2) / 12, mass * (sx**2 + sy**2) / 12)
    center = f"0 0 {sz / 2} 0 0 0"
    box = f"<box><size>{sx} {sy} {sz}</size></box>"
    hydro = ""
    if cfg.CONTACT_MODEL == "hydroelastic":
        hydro = (
            "<drake:compliant_hydroelastic/>"
            f"<drake:hydroelastic_modulus>{cfg.HYDROELASTIC_MODULUS}</drake:hydroelastic_modulus>"
        )
    friction = f"<drake:mu_dynamic>{mu}</drake:mu_dynamic><drake:mu_static>{mu}</drake:mu_static>"
    Path(path).write_text(
        f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <link name="{BOX_BODY}">
      <inertial>
        <pose>{center}</pose>
        <mass>{mass}</mass>
        <inertia><ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iyy}</iyy><iyz>0</iyz><izz>{izz}</izz></inertia>
      </inertial>
      <collision name="collision">
        <pose>{center}</pose>
        <geometry>{box}</geometry>
        <drake:proximity_properties>{hydro}{friction}</drake:proximity_properties>
      </collision>
      <visual name="visual">
        <pose>{center}</pose>
        <geometry>{box}</geometry>
        <material><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""
    )
    return Path(path)


def write_base_sdf(path: Path) -> Path:
    """The rolling base: x and y slides and a yaw joint carrying a thin visual-only plate (no collision).
    The arm is welded to `base_link`.  Joints have effort limits so Drake makes them actuated."""

    def link(name, mass, visual=""):
        i = mass * 0.05
        return (f'<link name="{name}"><inertial><mass>{mass}</mass><inertia><ixx>{i}</ixx><ixy>0</ixy><ixz>0</ixz>'
                f"<iyy>{i}</iyy><iyz>0</iyz><izz>{i}</izz></inertia></inertial>{visual}</link>")

    plate = ('<visual name="plate"><pose>0 0 0.005 0 0 0</pose><geometry><box><size>0.45 0.45 0.01</size></box>'
             "</geometry><material><diffuse>0.25 0.27 0.32 1</diffuse></material></visual>")

    def joint(name, kind, parent, child, axis, lim):
        return (f'<joint name="{name}" type="{kind}"><parent>{parent}</parent><child>{child}</child><axis>'
                f"<xyz>{axis}</xyz><limit><lower>-{lim}</lower><upper>{lim}</upper><effort>2000</effort></limit>"
                "</axis></joint>")

    Path(path).write_text(
        '<?xml version="1.0"?>\n<sdf version="1.7">\n  <model name="base">\n    '
        + link("slide_x", 5.0) + link("slide_y", 5.0) + link("base_link", 40.0, plate)
        + joint("base_x", "prismatic", "world", "slide_x", "1 0 0", 10)
        + joint("base_y", "prismatic", "slide_x", "slide_y", "0 1 0", 10)
        + joint("base_yaw", "revolute", "slide_y", "base_link", "0 0 1", 100)
        + "\n  </model>\n</sdf>\n"
    )
    return Path(path)


def api_hint(obj, *words) -> str:
    """Names on `obj` that contain any of `words` (case-insensitive): what this Drake version actually offers.
    Used in error messages, because the Drake calls here were written from memory."""
    names = sorted({n for n in dir(obj) if any(w.lower() in n.lower() for w in words) and not n.startswith("__")})
    return ", ".join(names) if names else "(nothing found)"


def contact_approximation(enum_class, setting: str):
    """The Drake enum value for config.CONTACT_APPROXIMATION ("sap" -> kSap, "lagged" -> kLagged ...), looked up
    only for the chosen setting, so a name this Drake version lacks (for example kTamsi) cannot break the others."""
    value = getattr(enum_class, "k" + setting.capitalize(), None)
    if value is None:
        options = [n[1:].lower() for n in dir(enum_class) if n.startswith("k") and n[1:2].isupper()]
        raise RuntimeError(f"CONTACT_APPROXIMATION = {setting!r} does not exist in this Drake. Available: {options} "
                           "(or \"default\" to leave Drake's own choice).")
    return value


class Sim:
    """The persistent simulator and everything about the robot's state.  See the module docstring."""

    def __init__(self, cfg, tower, friction: dict, record_replay: bool = False):
        self.cfg = cfg
        self.block_ids = sorted(tower.blocks)
        self.station = 0  # which station the base is at (the harness and movers read this)
        self._friction = dict(friction)  # HIDDEN truth: written into the SDF files, never exposed
        self.force_log = []
        self.actuator_notes = []
        self.kin = RobotKinematics(cfg)  # arm-only plant: forward/inverse kinematics in the robot frame
        self.mobile = cfg.BASE_MODE == "mobile"
        self.meshcat = _recording_meshcat() if record_replay else None
        if self.meshcat is not None:
            self.meshcat.Delete()
        self.asset_dir = Path(cfg.ASSET_DIR)
        self.asset_dir.mkdir(parents=True, exist_ok=True)

        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=cfg.PLANT_TIME_STEP)
        self._build_plant(tower)

        # ---- the controller loop: hand velocity -> joint velocities -> integrator -> desired arm state
        self.player = builder.AddSystem(MotionPlayer(cfg.FINGER_OPEN_COMMAND_M))
        arm_q = builder.AddSystem(FirstEntries(14, 7))
        joint_velocity = builder.AddSystem(JointVelocity(self.kin.plant))
        selector = builder.AddSystem(VelocitySelector(self.player))
        self.integrator = builder.AddSystem(Integrator(7))
        self.integrator.set_name("JointIntegrator")
        arm_desired = builder.AddSystem(PositionToState(7))
        builder.Connect(self.plant.get_state_output_port(self.iiwa), arm_q.get_input_port())
        builder.Connect(self.player.GetOutputPort("V_RG"), joint_velocity.get_input_port(0))
        builder.Connect(arm_q.get_output_port(), joint_velocity.get_input_port(1))
        builder.Connect(joint_velocity.get_output_port(), selector.GetInputPort("qdot_cartesian"))
        builder.Connect(self.player.GetOutputPort("qdot_joint"), selector.GetInputPort("qdot_joint"))
        builder.Connect(selector.get_output_port(), self.integrator.get_input_port())
        builder.Connect(self.integrator.get_output_port(), arm_desired.get_input_port())
        builder.Connect(arm_desired.get_output_port(), self._desired_state_port(self.iiwa, "arm"))

        # ---- the fingers
        finger_q = builder.AddSystem(OpeningToFingers())
        finger_desired = builder.AddSystem(PositionToState(2))
        builder.Connect(self.player.GetOutputPort("opening"), finger_q.get_input_port())
        builder.Connect(finger_q.get_output_port(), finger_desired.get_input_port())
        builder.Connect(finger_desired.get_output_port(), self._desired_state_port(self.wsg, "gripper"))

        # ---- the rolling base
        self.base_cmd = None
        if self.mobile:
            self.base_cmd = builder.AddSystem(BaseCommand(base_mod.base_xy_yaw(cfg, 0)))
            builder.Connect(self.base_cmd.get_output_port(), self._desired_state_port(self.base, "base"))

        if self.meshcat is not None:
            MeshcatVisualizer.AddToBuilder(builder, self.scene_graph, self.meshcat)

        self.diagram = builder.Build()
        self.simulator = Simulator(self.diagram)
        self.context = self.simulator.get_mutable_context()
        self._set_integrator(np.array(cfg.ARM_CARRY_Q, float))
        if self.meshcat is not None:
            self.meshcat.StartRecording(frames_per_second=32.0, set_visualizations_while_recording=False)

    # ------------------------------------------------------------------ writing the SDF files
    def _write_assets(self, tower) -> dict:
        cfg = self.cfg
        L, W, H = geo.block_dims(cfg)
        d = self.asset_dir
        fp = geo.tower_footprint_m(cfg) + 2 * cfg.PLATFORM_MARGIN_M
        assets = {
            "table": write_box_sdf(d / "table.sdf", "table", cfg.TABLE_SIZE_M, (0.55, 0.50, 0.45), 50.0,
                                   cfg.TABLE_FRICTION, cfg),
            "platform": write_box_sdf(d / "platform.sdf", "platform", (fp, fp, cfg.PLATFORM_HEIGHT_M),
                                      (0.40, 0.40, 0.45), 20.0, cfg.TABLE_FRICTION, cfg),
            "base": write_base_sdf(d / "base.sdf") if self.mobile else None,
            "blocks": {},
        }
        mass = geo.block_mass(cfg)
        for bid in self.block_ids:
            color = (0.80, 0.62, 0.40) if (bid // 3) % 2 == 0 else (0.68, 0.50, 0.30)
            assets["blocks"][bid] = write_box_sdf(d / f"block_{bid:02d}.sdf", f"block_{bid:02d}", (L, W, H), color,
                                                  mass, self._friction[bid], cfg)
        return assets

    # ------------------------------------------------------------------ building the plant by hand
    def _build_plant(self, tower) -> None:
        cfg, plant = self.cfg, self.plant
        parser = Parser(plant)
        ConfigureParser(parser)  # so package://manipulation/... resolves, as in the psets
        assets = self._write_assets(tower)

        def add_text(path):  # our own SDF files; returns the model instance
            return parser.AddModelsFromString(Path(path).read_text(), "sdf")[0]

        try:
            self.iiwa = parser.AddModelsFromUrl(IIWA_URL)[0]
            self.wsg = parser.AddModelsFromUrl(WSG_URL)[0]
        except RuntimeError as e:
            explain_model_download_error(e)  # exits with a helpful message if it applies
            raise
        table, platform = add_text(assets["table"]), add_text(assets["platform"])
        self.base = add_text(assets["base"]) if self.mobile else None
        blocks = {bid: add_text(p) for bid, p in assets["blocks"].items()}

        # ---- welds: the gripper onto the arm (same weld as the psets), the arm onto the base (or the world)
        X_wsg = RigidTransform(RollPitchYaw(np.radians([90.0, 0.0, 90.0])), [0.0, 0.0, 0.09])
        plant.WeldFrames(plant.GetFrameByName("iiwa_link_7", self.iiwa), plant.GetFrameByName("body", self.wsg), X_wsg)
        link0 = plant.GetFrameByName("iiwa_link_0", self.iiwa)
        if self.mobile:
            plant.WeldFrames(plant.GetFrameByName(BOX_BODY, self.base), link0,
                             RigidTransform([0.0, 0.0, cfg.BASE_MOUNT_HEIGHT_M]))
        else:
            plant.WeldFrames(plant.world_frame(), link0, RigidTransform(base_mod.station_pose(cfg, 0)))
        cx, cy = cfg.TOWER_CENTER_XY
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(BOX_BODY, table),
                         RigidTransform([0.0, 0.0, -cfg.TABLE_SIZE_M[2]]))
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(BOX_BODY, platform), RigidTransform([cx, cy, 0.0]))

        # ---- physics settings (before Finalize)
        if cfg.CONTACT_MODEL == "point":
            plant.set_contact_model(ContactModel.kPoint)
        if cfg.CONTACT_APPROXIMATION != "default":
            plant.set_discrete_contact_approximation(
                contact_approximation(DiscreteContactApproximation, cfg.CONTACT_APPROXIMATION))
        plant.set_gravity_enabled(self.iiwa, False)  # ideal gravity-compensated arm; the gripper keeps its weight

        # ---- PD-controlled joints: stiffness and damping per actuated joint
        self._set_gains(self.iiwa, IIWA_JOINTS, cfg.ARM_KP, cfg.ARM_KD, "arm")
        self._set_gains(self.wsg, WSG_JOINTS, [cfg.FINGER_KP] * 2, [cfg.FINGER_KD] * 2, "gripper")
        if self.mobile:
            self._set_gains(self.base, BASE_JOINTS, cfg.BASE_KP, cfg.BASE_KD, "base")

        plant.Finalize()

        # ---- where everything starts (Drake only allows this AFTER Finalize, as in the psets)
        plant.SetDefaultPositions(self.iiwa, np.array(cfg.ARM_CARRY_Q, float))
        plant.SetDefaultPositions(self.wsg, [-cfg.FINGER_OPEN_COMMAND_M / 2, cfg.FINGER_OPEN_COMMAND_M / 2])
        if self.mobile:
            plant.SetDefaultPositions(self.base, np.array(base_mod.base_xy_yaw(cfg, 0), float))
        for bid, inst in blocks.items():
            plant.SetDefaultPositions(inst, geo.free_body_q(tower.blocks[bid].X_WB))

    def _set_gains(self, instance, joint_names, kp, kd, what: str) -> None:
        """PD stiffness and damping for each named joint.  Each joint's actuator is found the way the pset does it
        (mobile_base_ik.py): walk the plant's actuators and compare actuator.joint() with the joint.  The actuators
        come from the model files; nothing is added here, so a missing one is an error, not something we invent."""
        plant = self.plant
        if not (len(joint_names) == len(kp) == len(kd)):
            raise RuntimeError(f"The {what} has {len(joint_names)} joints but {len(kp)} stiffness and "
                               f"{len(kd)} damping values in config.py.")
        actuators = [plant.get_joint_actuator(i) for i in plant.GetJointActuatorIndices()]
        for name, p, d in zip(joint_names, kp, kd):
            joint = plant.GetJointByName(name, instance)
            found = [a for a in actuators if a.joint() == joint]
            if not found:
                raise RuntimeError(f"The {what}'s joint {name!r} has no actuator in its model file. Joints that do: "
                                   f"{[a.joint().name() for a in actuators]}")
            try:
                found[0].set_controller_gains(PdControllerGains(p=float(p), d=float(d)))
            except (AttributeError, TypeError) as e:
                raise RuntimeError(
                    f"Could not set PD gains on the {what}'s joints: {e}\n"
                    f"JointActuator offers: {api_hint(found[0], 'gain', 'pd', 'control')}\n"
                    f"PdControllerGains offers: {api_hint(PdControllerGains, 'p', 'd')}") from e
        self.actuator_notes.append(f"{what}: {len(joint_names)} PD joints")

    def _desired_state_port(self, instance, what: str):
        try:
            return self.plant.get_desired_state_input_port(instance)
        except AttributeError as e:
            raise RuntimeError(f"The plant has no desired-state port for the {what}: {e}\n"
                               f"MultibodyPlant offers: {api_hint(self.plant, 'desired', 'actuation')}") from e

    # ------------------------------------------------------------------ time
    def time(self) -> float:
        return float(self.context.get_time())

    def advance(self, seconds: float) -> None:
        self.simulator.AdvanceTo(self.time() + seconds)

    def hold(self, seconds: float) -> None:
        """Stand still (arm and fingers hold their last command) for `seconds`."""
        self.player.stop()
        self.advance(seconds)

    # ------------------------------------------------------------------ reading the world (TRUE state)
    def _pc(self):
        return self.plant.GetMyContextFromRoot(self.context)

    def block_poses(self) -> dict:
        """True 4x4 world poses of every block (evaluation side and tower bookkeeping)."""
        pc = self._pc()
        out = {}
        for bid in self.block_ids:
            inst = self.plant.GetModelInstanceByName(f"block_{bid:02d}")
            body = self.plant.GetBodyByName(BOX_BODY, inst)
            out[bid] = np.array(self.plant.EvalBodyPoseInWorld(pc, body).GetAsMatrix4())
        return out

    def arm_q(self) -> np.ndarray:
        return np.array(self.plant.GetPositions(self._pc(), self.iiwa), dtype=float)

    def base_q(self):
        """(x, y, yaw) of the base (yaw is unwrapped)."""
        if not self.mobile:
            return base_mod.base_xy_yaw(self.cfg, 0)
        return tuple(float(v) for v in self.plant.GetPositions(self._pc(), self.base))

    def X_WR(self) -> np.ndarray:
        """Pose of the robot base frame (arm mounting flange) in the world, from the simulator."""
        body = self.plant.GetBodyByName("iiwa_link_0", self.iiwa)
        return np.array(self.plant.EvalBodyPoseInWorld(self._pc(), body).GetAsMatrix4())

    def wrist_force_raw(self) -> np.ndarray:
        """[Fx Fy Fz Tx Ty Tz] reaction at the weld between the arm and the gripper, in the GRIPPER frame
        (measured on our machines).  Includes the gripper's own weight.  Raw: no compensation, no noise."""
        if not hasattr(self, "_wrist_joint"):
            self._wrist_joint = None
            for ji in self.plant.GetJointIndices():
                j = self.plant.get_joint(ji)
                if (j.type_name() == "weld" and j.parent_body().model_instance() == self.iiwa
                        and j.child_body().model_instance() == self.wsg):
                    self._wrist_joint = int(ji)
            if self._wrist_joint is None:
                raise RuntimeError("Could not find the weld joint between the iiwa and the gripper.")
        F = self.plant.get_reaction_forces_output_port().Eval(self._pc())[self._wrist_joint]
        return np.concatenate([np.asarray(F.translational()), np.asarray(F.rotational())])

    def gripper_mass(self) -> float:
        return float(self.plant.CalcTotalMass(self._pc(), [self.wsg]))

    def describe(self) -> str:
        """One line for the checks report: how the plant is configured."""
        try:
            approx = str(self.plant.get_discrete_contact_approximation())
        except Exception:  # noqa: BLE001
            approx = "unknown"
        return (f"time step {self.cfg.PLANT_TIME_STEP} s, discrete contact approximation {approx}, "
                f"contact model {self.cfg.CONTACT_MODEL}, base {'rolling joints' if self.mobile else 'fixed weld'}; "
                f"actuators: " + "; ".join(self.actuator_notes))

    # ------------------------------------------------------------------ changing the world
    def teleport_block(self, block_id: int, X_WB) -> None:
        """FAKE MOVER ONLY: place a block at a pose, at rest."""
        inst = self.plant.GetModelInstanceByName(f"block_{block_id:02d}")
        ctx = self.plant.GetMyMutableContextFromRoot(self.context)
        self.plant.SetPositions(ctx, inst, geo.free_body_q(np.asarray(X_WB, float)))
        self.plant.SetVelocities(ctx, inst, np.zeros(6))

    def _set_integrator(self, q) -> None:
        self.integrator.set_integral_value(self.integrator.GetMyContextFromRoot(self.context), q)

    # ------------------------------------------------------------------ used by the executor
    def start_cartesian(self, v_traj, finger_traj, q_now) -> None:
        self._set_integrator(q_now)  # restart from the MEASURED angles: the controller is feedforward
        self.player.start_cartesian(v_traj, finger_traj, self.time())

    def start_joint(self, q_now, q_target, T: float, opening: float) -> None:
        self._set_integrator(q_now)
        self.player.start_joint(np.asarray(q_target) - np.asarray(q_now), T, opening, self.time())

    def set_base_target(self, x, y, yaw) -> None:
        self.base_cmd.set(x, y, yaw)

    def advance_logged(self, seconds: float) -> None:
        """Advance, sampling the wrist force every SAMPLE_DT_S when config.ENABLE_WRIST_FORCE_LOG is on."""
        if not self.cfg.ENABLE_WRIST_FORCE_LOG:
            self.advance(seconds)
            return
        end = self.time() + seconds
        while self.time() < end - 1e-9:
            self.simulator.AdvanceTo(min(self.time() + self.cfg.SAMPLE_DT_S, end))
            self.force_log.append([self.time(), *self.wrist_force_raw()])

    # ------------------------------------------------------------------ outputs
    def save_replay(self, path) -> bool:
        """Write the recorded trial as one self-contained HTML file, playable at normal speed in a browser."""
        if self.meshcat is None:
            return False
        self.meshcat.StopRecording()
        self.meshcat.PublishRecording()
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(self.meshcat.StaticHtml())
        return True

    def save_force_log(self, path) -> bool:
        if not self.force_log:
            return False
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"])
            w.writerows(self.force_log)
        return True
