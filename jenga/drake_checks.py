"""Checks that need Drake.  Called by run_checks.py.  (Drake code: UNTESTED when written; these checks are how
your machine tells us which parts work.)

Each check takes (cfg, shared) and returns (status, headline, detail) with status "PASS", "FAIL" or "WARN".
`shared` holds the one Sim reused by several checks, so the scene is built once.
"""

import time
from types import SimpleNamespace

import numpy as np

from jenga import base as base_mod
from jenga import geometry as geo

RA = np.column_stack([[0, 1, 0], [1, 0, 0], [0, 0, -1]]).astype(float)  # hand level, G x = +y_R, G y = +x_R
RB = np.column_stack([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).astype(float)  # the same grasp, hand rolled 180 degrees


def with_overrides(cfg, **over):
    d = {k: getattr(cfg, k) for k in dir(cfg) if k.isupper()}
    d.update(over)
    return SimpleNamespace(**d)


def _sim(cfg, shared):
    """Build (once) and return the shared simulator, or raise the build error again."""
    if "sim" in shared:
        return shared["sim"]
    if "sim_error" in shared:
        raise RuntimeError(f"the scene did not build (see the 'scene' check): {shared['sim_error']}")
    from jenga.scene import Sim
    from jenga.tower import TowerState

    tower = TowerState(cfg)
    friction = geo.assign_friction(cfg, cfg.BASE_SEED)
    try:
        t0 = time.time()
        shared["sim"] = Sim(with_overrides(cfg, RECORD_REPLAY=False), tower, friction, record_replay=False)
        shared["build_s"] = time.time() - t0
    except Exception as e:  # noqa: BLE001
        shared["sim_error"] = f"{type(e).__name__}: {e}"
        raise
    shared["tower"] = tower
    return shared["sim"]


def _max_move(sim, before):
    after = sim.block_poses()
    return max(float(np.linalg.norm(after[b][:3, 3] - before[b][:3, 3])) for b in before)


# ---------------------------------------------------------------- 1. reachability of every layer (arm only, cheap)
def check_reachability(cfg, shared):
    from pydrake.all import RigidTransform, RotationMatrix
    from jenga.kinematics import RobotKinematics

    kin = shared.setdefault("kin", RobotKinematics(cfg))
    L, W, H = geo.block_dims(cfg)
    gap = cfg.BLOCK_GAP_M
    d_b = cfg.STATION_STANDOFF_M - (W + gap)  # near edge block centre, along the reach direction
    offsets = {"approach": -0.10, "grasp": 0.0, "pulled out": -0.07, "place at middle slot": W + gap}
    lines, bad = [], []
    q_ref = np.array(cfg.ARM_CARRY_Q, float)
    for i in range(cfg.REACH_CHECK_LAYERS + 1):  # the last one is the placement level on top
        z = cfg.PLATFORM_HEIGHT_M + i * (H + gap) + H / 2 - cfg.BASE_MOUNT_HEIGHT_M
        best, best_detail = -1.0, {}
        for R in (RA, RB):
            margins = {}
            for pose_name, dx in offsets.items():
                X = RigidTransform(RotationMatrix(R), [d_b + dx - cfg.PAD_CENTRE_FROM_G_M, 0.0, z])
                q = kin.solve_ik_best(X, q_ref, tries=8)
                margins[pose_name] = -1.0 if q is None else float(np.degrees(kin.joint_limit_margin(q)))
            worst = min(margins.values())
            if worst > best or not best_detail:
                best, best_detail = worst, margins
        limiting = min(best_detail, key=best_detail.get)
        tag = "layer" if i < cfg.REACH_CHECK_LAYERS else "place level"
        text = "UNREACHABLE" if best < 0 else f"{best:.1f} deg"
        per_pose = ", ".join(f"{k}: " + ("none" if v < 0 else f"{v:.0f}") for k, v in best_detail.items())
        lines.append(f"{tag} {i}: pad height {z:.3f} m above the robot base -> worst joint margin {text} (limiting: {limiting}) [{per_pose}]")
        if best < cfg.MIN_JOINT_MARGIN_DEG:
            bad.append(i)
    note = (f"Level hand at the near edge block from a station {cfg.STATION_STANDOFF_M} m from the tower centre; best of two hand rolls; "
            f"IK only (no collisions); wanted >= {cfg.MIN_JOINT_MARGIN_DEG} deg.  Margins in degrees per pose in brackets.\n"
            "If this fails: change STATION_STANDOFF_M (farther out usually helps) or PLATFORM_HEIGHT_M.")
    for ln in lines:
        print("   " + ln)
    detail = "\n".join(lines + [note])
    if bad:
        return "FAIL", f"levels {bad} are not reachable with enough margin", detail
    return "PASS", f"all {cfg.REACH_CHECK_LAYERS} layers and the placement level reachable", detail


# ---------------------------------------------------------------- 2. scene builds, tower stands still
def check_scene_and_static(cfg, shared):
    sim = _sim(cfg, shared)
    before = sim.block_poses()
    t0, s0 = time.time(), sim.time()
    sim.hold(cfg.STATIC_CHECK_S)
    wall, simd = time.time() - t0, sim.time() - s0
    move = _max_move(sim, before)
    shared["speed"] = simd / wall if wall > 0 else float("nan")
    detail = (f"scene built in {shared['build_s']:.1f} s; {simd:.1f} s simulated in {wall:.1f} s wall "
              f"({shared['speed']:.2f}x real time); largest block movement {move * 1000:.2f} mm "
              f"(limit {cfg.STATIC_MAX_MOVE_M * 1000:.1f} mm); {sim.describe()}")
    if move > cfg.STATIC_MAX_MOVE_M:
        return "FAIL", f"the tower moved {move * 1000:.2f} mm while standing still", detail
    return "PASS", f"the tower stands still ({move * 1000:.2f} mm in {cfg.STATIC_CHECK_S:.0f} s)", detail


# ---------------------------------------------------------------- 3. gripper numbers read from the model
def check_gripper(cfg, shared):
    from pydrake.geometry import Box, Role

    sim = _sim(cfg, shared)
    plant, pc = sim.plant, sim._pc()
    G = plant.GetBodyByName("body", sim.wsg)
    X_WG = plant.EvalBodyPoseInWorld(pc, G)
    inner = {}
    for name in ("left_finger", "right_finger"):
        body = plant.GetBodyByName(name, sim.wsg)
        fid = plant.GetBodyFrameIdOrThrow(body.index())
        inspector = sim.scene_graph.model_inspector()
        X_WF = plant.EvalBodyPoseInWorld(pc, body)
        for gid in inspector.GetGeometries(fid, Role.kProximity):
            shape = inspector.GetShape(gid)
            if isinstance(shape, Box) and np.allclose(sorted(shape.size()), [0.003, 0.02, 0.02], atol=1e-6):
                X_GP = X_WG.InvertAndCompose(X_WF @ inspector.GetPoseInFrame(gid))
                half = np.abs(X_GP.rotation().matrix()[0, :]) @ (np.asarray(shape.size()) / 2)
                cx = X_GP.translation()[0]
                inner[name] = cx + half if cx < 0 else cx - half
    if len(inner) != 2:
        return "FAIL", "could not find both fingertip pads in the model", f"found: {inner}"
    gap = float(inner["right_finger"] - inner["left_finger"])
    want = geo.pad_gap_m(cfg)
    L, _, _ = geo.block_dims(cfg)
    detail = (f"pad gap read from the model: {gap * 1000:.2f} mm at the default opening "
              f"({cfg.FINGER_OPEN_COMMAND_M * 1000:.0f} mm command); config expects {want * 1000:.2f} mm; "
              f"block length {L * 1000:.2f} mm -> clearance {(gap - L) / 2 * 1000:.2f} mm per side")
    if abs(gap - want) > 0.0005:
        return "FAIL", "pad gap in the model differs from config (more than 0.5 mm)", detail
    if gap < L:
        return "FAIL", "the block does not fit between the open pads", detail
    return "PASS", f"pad gap {gap * 1000:.1f} mm; the block fits with {(gap - L) / 2 * 1000:.2f} mm per side", detail


# ---------------------------------------------------------------- 4. robot-frame consistency
def check_robot_frame(cfg, shared):
    sim = _sim(cfg, shared)
    q = sim.arm_q()
    X_RG = sim.kin.gripper_pose(q).GetAsMatrix4()
    pred = sim.X_WR() @ X_RG
    G = sim.plant.GetBodyByName("body", sim.wsg)
    actual = sim.plant.EvalBodyPoseInWorld(sim._pc(), G).GetAsMatrix4()
    err = float(np.linalg.norm(pred[:3, 3] - actual[:3, 3]))
    rot = float(np.linalg.norm(pred[:3, :3] - actual[:3, :3]))
    detail = f"hand position error {err * 1000:.2f} mm, rotation error {rot:.4f}  (X_WR from the simulator times arm-only forward kinematics)"
    if err > 0.002 or rot > 0.01:
        return "FAIL", "the arm-only plant and the full scene disagree about where the hand is", detail
    return "PASS", f"the robot frame is consistent ({err * 1000:.2f} mm)", detail


# ---------------------------------------------------------------- 5. wrist force at rest
def check_wrist_force(cfg, shared):
    sim = _sim(cfg, shared)
    sim.hold(1.0)
    F = sim.wrist_force_raw()
    m = sim.gripper_mass()
    weight = m * 9.81
    mag = float(np.linalg.norm(F[:3]))
    detail = (f"raw [Fx Fy Fz Tx Ty Tz] = {np.round(F, 2).tolist()}; |F| = {mag:.2f} N; gripper mass {m:.3f} kg "
              f"-> weight {weight:.2f} N; config's measured value {cfg.WRIST_REST_FORCE_N} N")
    if abs(mag - weight) > 0.15 * weight:
        return "FAIL", f"|F| = {mag:.1f} N is not about the hand's weight ({weight:.1f} N)", detail
    return "PASS", f"raw wrist force {mag:.1f} N matches the hand's weight", detail


# ---------------------------------------------------------------- 6. executor: motions on one persistent simulator
def check_executor(cfg, shared):
    from pydrake.all import RigidTransform
    from jenga import executor
    from jenga.waypoint import Waypoint

    sim = _sim(cfg, shared)
    before = sim.block_poses()
    q0 = sim.arm_q()
    X0 = sim.kin.gripper_pose(q0)
    t0 = sim.time()
    down = RigidTransform(X0.rotation(), X0.translation() + np.array([0.0, 0.0, -0.05]))
    r1 = executor.run_waypoints(sim, [Waypoint(down)], cfg.WAYPOINT_SPEED_MS, "check: down 5 cm")
    r2 = executor.run_waypoints(sim, [Waypoint(X0)], cfg.WAYPOINT_SPEED_MS, "check: back up")
    # a joint-space move too
    q1 = q0.copy()
    q1[0] += 0.3
    r3 = executor.move_arm_to_q(sim, q1, label="check: joint move")
    r4 = executor.move_arm_to_q(sim, q0, label="check: joint move back")
    back = float(np.linalg.norm(sim.kin.gripper_pose(sim.arm_q()).translation() - X0.translation()))
    move = _max_move(sim, before)
    detail = (f"down 5 cm: ended {r1.tracking_error_m * 1000:.1f} mm from target; back up: {r2.tracking_error_m * 1000:.1f} mm; "
              f"after a joint move and back the hand is {back * 1000:.1f} mm from where it started; "
              f"simulated time advanced {sim.time() - t0:.1f} s across 4 motions in ONE simulator; "
              f"smallest joint margin {np.degrees(min(r1.joint_margin_rad, r2.joint_margin_rad, r3.joint_margin_rad, r4.joint_margin_rad)):.1f} deg; "
              f"largest block movement {move * 1000:.2f} mm")
    if max(r1.tracking_error_m, r2.tracking_error_m, back) > cfg.TRACKING_WARN_M:
        return "FAIL", "the hand did not reach its targets within tolerance", detail
    return "PASS", "four motions ran in one persistent simulator and reached their targets", detail


# ---------------------------------------------------------------- 7. base stations
def check_base(cfg, shared):
    from jenga import executor

    if cfg.BASE_MODE != "mobile":
        return "WARN", "skipped: BASE_MODE is 'fixed'", "Set BASE_MODE = 'mobile' in config.py to check the rolling base."
    sim = _sim(cfg, shared)
    n = len(cfg.STATION_ANGLES_DEG)
    order = [k for k in (1, 2) if k < n] + [0]
    before = sim.block_poses()
    lines, worst_pos, worst_ang, worst_standoff = [], 0.0, 0.0, 0.0
    for k in order:
        t0 = time.time()
        executor.move_base_to_station(sim, k)
        want = base_mod.station_pose(cfg, k)
        got = sim.X_WR()
        pos = float(np.linalg.norm(got[:3, 3] - want[:3, 3]))
        yaw_got = float(np.arctan2(got[1, 0], got[0, 0]))
        yaw_want = float(np.arctan2(want[1, 0], want[0, 0]))
        ang = abs((yaw_got - yaw_want + np.pi) % (2 * np.pi) - np.pi)
        cx, cy = cfg.TOWER_CENTER_XY
        centre_R = base_mod.to_robot_frame(got, geo.make_pose(cx, cy, 0.0))
        standoff = float(np.hypot(centre_R[0, 3], centre_R[1, 3]))
        worst_pos, worst_ang = max(worst_pos, pos), max(worst_ang, ang)
        worst_standoff = max(worst_standoff, abs(standoff - cfg.STATION_STANDOFF_M))
        lines.append(f"station {k}: position error {pos * 1000:.1f} mm, yaw error {np.degrees(ang):.2f} deg, "
                     f"tower centre {standoff:.3f} m from the base axis, took {time.time() - t0:.0f} s wall")
    move = _max_move(sim, before)
    lines.append(f"largest block movement during all base moves: {move * 1000:.2f} mm (limit {cfg.STATIC_MAX_MOVE_M * 1000:.1f} mm)")
    detail = "\n".join(lines)
    if worst_pos > 0.005 or worst_ang > np.radians(1.0) or worst_standoff > 0.005:
        return "FAIL", "the base did not end where commanded (more than 5 mm or 1 deg)", detail
    if move > cfg.STATIC_MAX_MOVE_M:
        return "FAIL", f"rolling the base disturbed the tower ({move * 1000:.2f} mm)", detail
    return "PASS", f"base rolled through stations {order} without disturbing the tower", detail


# ---------------------------------------------------------------- 8. fake-mover trial end to end, and the guard
def check_fake_trial(cfg, shared):
    from pathlib import Path

    from jenga import mover as mover_mod
    from jenga.evaluation import harness, results

    c = with_overrides(cfg, MAX_TURNS=2, MOVER="fake", N_TRIALS=1, BASE_MODE=cfg.BASE_MODE)
    out = Path(cfg.FAKE_RUNS_DIR) / "_checks"
    for f in out.glob("*") if out.exists() else []:
        f.unlink()
    lines = []
    summary = harness.run_trial(c, 0, mover_mod.FakeMover(), out, log=lambda s: lines.append(s))
    try:
        results.read_trials(Path(cfg.FAKE_RUNS_DIR), cfg)
        guard = False
    except results.FakeRunsError:
        guard = True
    replay = summary.get("replay_file", "")
    ok = summary["mover"] == "fake" and guard and summary["turns_attempted"] >= 1
    detail = "\n".join(lines) + f"\nreplay file: {replay or '(none; RECORD_REPLAY off)'}; the results script refused the fake folder: {guard}"
    if cfg.RECORD_REPLAY and not replay:
        return "FAIL", "no replay file was written", detail
    if not ok:
        return "FAIL", "fake trial or the fake-run guard did not behave", detail
    return "PASS", f"fake trial ran {summary['turns_attempted']} turn(s), tagged fake; results guard works", detail


# ---------------------------------------------------------------- optional: point vs hydroelastic (the reminder in DECISIONS.md)
def check_compare_contact(cfg, shared):
    from jenga.scene import Sim
    from jenga.tower import TowerState

    rows = []
    for model in ("hydroelastic", "point"):
        c = with_overrides(cfg, CONTACT_MODEL=model, RECORD_REPLAY=False)
        tower = TowerState(c)
        sim = Sim(c, tower, geo.assign_friction(c, c.BASE_SEED), record_replay=False)
        before = sim.block_poses()
        t0 = time.time()
        sim.hold(c.STATIC_CHECK_S)
        wall = time.time() - t0
        rows.append((model, c.STATIC_CHECK_S / wall, _max_move(sim, before)))
    detail = "\n".join(f"{m:13s}: {s:.2f}x real time, largest block movement {mv * 1000:.2f} mm" for m, s, mv in rows)
    detail += "\nIf point contact holds the tower still (movement under the limit) and is faster, switch CONTACT_MODEL " \
              "after also checking a real pull (DECISIONS.md)."
    return "PASS", "compared (read the numbers)", detail
