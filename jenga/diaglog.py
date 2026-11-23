"""Development log: what happened in each STEP of a turn (base rolls, push, pull, pedestal, regrab, place), so a
failed turn shows which step went wrong and by how much, not only the turn's outcome.  Off for results.

config.DIAG_LOG_LEVEL: 0 off; 1 one row per step in steps.csv; 2 also one row per block per step in
step_blocks.csv.  config.DIAG_LOG_FEATURES picks the column groups of steps.csv, and "waypoints":
  waypoints   also one row per waypoint of each plan (column `waypoint` = its index in the plan, 0 = the joint
              move), written when the hand is due there; the step's own row (waypoint blank) follows.  Moves in
              a waypoint row are since the previous waypoint.  The simulator stops at each waypoint (no effect
              on the result, D-24)
  block       the pulled block against its pose at the START of the turn, in that pose's own axes: along its long
              axis (for a push: how far it came out), sideways, up (mm); yaw and tilt (deg)
  neighbours  how far the OTHER blocks moved in THIS step: the worst one (id, layer, mm, tilt), the worst in the
              same layer, the layer above, the layer below, any other layer; and the worst since the turn began
  hand        the hand's distance from its last waypoint at the end of a plan (the executor's tracking error),
              and the block's centre in the gripper frame G (x finger axis, y reach axis, F-02): how it sits in
              the pads when held.  Waypoint rows add where the hand is against that waypoint, in the waypoint's
              G frame (err_x/y/z_mm, actual minus target), and the commanded finger opening
  joints      smallest arm-joint margin to a limit (deg), and the seven arm angles q1..q7 (deg).  A plan's first move
              is a straight line in joint angles (jenga/executor.py), so two rows' angles give the whole swing.
              With CHECK_SWING_CLEARANCE, the step's swing choice (jenga/clearance.py): which IK solution was taken
              (1 = the one taken without the check) of how many, its clearance (mm) and the nearest obstacle

Uses simulator truth: evaluation side only, never seen by a strategy (CLAUDE.md rule 5).
`world` is jenga.scene.Sim or a stand-in with block_poses, arm_q, X_WR, kin, time.
"""

import numpy as np

from jenga import base

_GROUPS = {
    "block": ["block_along_mm", "block_side_mm", "block_up_mm", "block_yaw_deg", "block_tilt_deg"],
    "neighbours": ["worst_id", "worst_layer", "worst_mm", "worst_tilt_deg", "same_layer_mm", "above_mm",
                   "below_mm", "other_layers_mm", "worst_since_turn_mm"],
    "hand": ["tracking_mm", "in_hand_x_mm", "in_hand_y_mm", "in_hand_z_mm", "err_x_mm", "err_y_mm", "err_z_mm",
             "opening_cmd_mm"],
    "joints": ["joint_margin_deg"] + [f"q{i}_deg" for i in range(1, 8)]
              + ["swing_tried", "swing_options", "swing_clear_mm", "swing_nearest"],
}
BLOCK_COLUMNS = ["step", "block", "layer", "step_move_mm", "turn_move_mm", "turn_tilt_deg"]


def _mat(X) -> np.ndarray:
    return np.asarray(X.GetAsMatrix4() if hasattr(X, "GetAsMatrix4") else X, dtype=float)


def _tilt_deg(R0, R1) -> float:
    """Angle between the two poses' z axes (how far the block tipped)."""
    return float(np.degrees(np.arccos(np.clip(R0[:, 2] @ R1[:, 2], -1.0, 1.0))))


def _yaw_deg(R0, R1) -> float:
    """Turn about the first pose's z axis."""
    R = R0.T @ R1
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def _r(x, n=2):
    return round(float(x), n)


def step_columns(features) -> list:
    """steps.csv columns (after the harness's trial, turn, block_id) for these feature groups, in a fixed order."""
    return ["step", "waypoint", "sim_t", "step_s", "error"] + [c for g in _GROUPS if g in features for c in _GROUPS[g]]


class StepLog:
    """Rows for one turn.  The mover calls begin_step() before each step and step(name, result) after it; with
    "waypoints" on, the executor calls waypoint(name, i, wp) at each waypoint of a plan in between."""

    def __init__(self, world, tower, block_id, cfg):
        self.world, self.block = world, block_id
        self.features = set(cfg.DIAG_LOG_FEATURES)
        self.level = cfg.DIAG_LOG_LEVEL
        self.columns = step_columns(self.features)
        self.layers = {b: r.layer for b, r in tower.blocks.items()}  # at the start of the turn
        self.layer = self.layers[block_id]
        self.start = world.block_poses()
        self.per_waypoint = "waypoints" in self.features
        self.prev, self.t_prev = self.start, world.time()  # at the end of the last step
        self.prev_wp, self.t_prev_wp = self.prev, self.t_prev  # at the last waypoint row
        self.rows, self.block_rows = [], []

    def begin_step(self) -> None:
        self.world.last_swing = None  # the executor fills it when this step's swing is checked

    def _row(self, name, wp_index, now, ref, t_ref, result=None, error="", wp=None) -> dict:
        """One row: `now` against `ref` (poses at t_ref) for the per-step moves, against the turn start otherwise."""
        w = self.world
        row = dict(step=name, waypoint=wp_index, sim_t=_r(w.time()), step_s=_r(w.time() - t_ref), error=error)
        P0, P = self.start[self.block], now[self.block]
        if "block" in self.features:
            d = P0[:3, :3].T @ (P[:3, 3] - P0[:3, 3])
            row.update(block_along_mm=_r(d[0] * 1000), block_side_mm=_r(d[1] * 1000), block_up_mm=_r(d[2] * 1000),
                       block_yaw_deg=_r(_yaw_deg(P0[:3, :3], P[:3, :3])),
                       block_tilt_deg=_r(_tilt_deg(P0[:3, :3], P[:3, :3])))
        others = [b for b in now if b != self.block]
        move = {b: float(np.linalg.norm(now[b][:3, 3] - ref[b][:3, 3])) for b in others}
        if "neighbours" in self.features and others:
            worst = max(others, key=move.get)

            def group(pick):
                return _r(max([move[b] for b in others if pick(self.layers[b])], default=0.0) * 1000)

            row.update(
                worst_id=worst, worst_layer=self.layers[worst], worst_mm=_r(move[worst] * 1000),
                worst_tilt_deg=_r(_tilt_deg(ref[worst][:3, :3], now[worst][:3, :3])),
                same_layer_mm=group(lambda l: l == self.layer),
                above_mm=group(lambda l: l is not None and l == self.layer + 1),
                below_mm=group(lambda l: l is not None and l == self.layer - 1),
                other_layers_mm=group(lambda l: l is None or abs(l - self.layer) > 1),
                worst_since_turn_mm=_r(max(np.linalg.norm(now[b][:3, 3] - self.start[b][:3, 3]) for b in others) * 1000))
        if "hand" in self.features:
            X_RG = _mat(w.kin.gripper_pose(w.arm_q()))
            p = (np.linalg.inv(X_RG) @ base.to_robot_frame(w.X_WR(), P))[:3, 3]
            row.update(tracking_mm=_r(result.tracking_error_m * 1000) if result is not None else "",
                       in_hand_x_mm=_r(p[0] * 1000), in_hand_y_mm=_r(p[1] * 1000), in_hand_z_mm=_r(p[2] * 1000))
            row.update(err_x_mm="", err_y_mm="", err_z_mm="", opening_cmd_mm="")  # waypoint rows only
            if wp is not None:
                e = (np.linalg.inv(_mat(wp.X_RG)) @ X_RG)[:3, 3]
                row.update(err_x_mm=_r(e[0] * 1000), err_y_mm=_r(e[1] * 1000), err_z_mm=_r(e[2] * 1000),
                           opening_cmd_mm=_r(wp.opening * 1000) if wp.opening is not None else "")
        if "joints" in self.features:
            q = w.arm_q()
            row["joint_margin_deg"] = _r(np.degrees(w.kin.joint_limit_margin(q)), 1)
            row.update({f"q{i}_deg": _r(np.degrees(v), 1) for i, v in enumerate(q, 1)})
            swing = getattr(w, "last_swing", None) if wp_index in ("", 0) else None
            row.update({k: (swing or {}).get(k, "") for k in ("swing_tried", "swing_options", "swing_clear_mm",
                                                              "swing_nearest")})
        return row

    def waypoint(self, name: str, i: int, wp) -> None:
        """Record the hand and the blocks when the hand is due at waypoint i of this step's plan."""
        now = self.world.block_poses()
        self.rows.append(self._row(name, i, now, self.prev_wp, self.t_prev_wp, wp=wp))
        self.prev_wp, self.t_prev_wp = now, self.world.time()

    def step(self, name: str, result=None, error: str = "") -> None:
        """Record the step that just ended.  result: the executor's PhaseResult for a plan, None for a base roll."""
        w, now = self.world, self.world.block_poses()
        row = self._row(name, "", now, self.prev, self.t_prev, result=result, error=error)
        self.rows.append(row)
        if self.level >= 2:
            for b in now:
                self.block_rows.append(dict(
                    step=name, block=b, layer=self.layers[b],
                    step_move_mm=_r(np.linalg.norm(now[b][:3, 3] - self.prev[b][:3, 3]) * 1000),
                    turn_move_mm=_r(np.linalg.norm(now[b][:3, 3] - self.start[b][:3, 3]) * 1000),
                    turn_tilt_deg=_r(_tilt_deg(self.start[b][:3, :3], now[b][:3, :3]))))
        self.prev, self.t_prev = now, w.time()
        self.prev_wp, self.t_prev_wp = now, w.time()
