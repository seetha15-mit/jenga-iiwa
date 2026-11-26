"""The evaluation harness: runs trials, logs every turn, ends a trial on collapse.

One trial = one persistent simulator.  Turn loop:
    1. read the true poses into the tower state
    2. candidates = legal pulls (tower rule) that the extraction method can physically take
    3. the strategy picks one (the FIRST pick of every trial is random, whatever the strategy: course rule)
    4. the mover carries out the turn (real steps, or the fake teleport)
    5. EVALUATION: classify what happened to the block (true poses), run the collapse detector
    6. log a CSV row; end the trial on collapse, too many failures in a row, or MAX_TURNS

Seeds: trial i uses seed = BASE_SEED + i.  The seed fixes the first block (a random pick from its own stream,
so every strategy gets the same first block) and the hidden friction draw.  A separate stream drives the
strategy's own randomness.

Fake runs: if the mover's TAG is "fake", the output goes to config.FAKE_RUNS_DIR, every CSV row says
mover = fake, and jenga/evaluation/results.py refuses to build a results table from them.
"""

import csv
import time
from pathlib import Path

import numpy as np

from jenga import clearance, diaglog, geometry as geo, methods, perceive, strategies
from jenga.errors import MotionFailed
from jenga.evaluation import collapse, outcomes
from jenga.tower import TowerState

TURN_COLUMNS = [
    "trial", "seed", "strategy", "extraction_method", "mover", "friction_mode", "n_layers_start", "block_scale",
    "contact_model", "turn", "block_id", "layer", "slot", "target_layer", "target_slot", "station_pull",
    "station_place", "outcome", "collapse", "collapse_reason", "mover_error", "arm_collisions_unchecked",
    "sim_seconds", "wall_seconds", "place_across_mm", "place_along_mm", "place_height_mm",
    "perception_unseen", "perception_max_height_mm", "perception_max_pos_mm",
]
TRIAL_COLUMNS = [
    "trial", "seed", "strategy", "extraction_method", "mover", "friction_mode", "n_layers_start", "block_scale",
    "contact_model", "turns_attempted", "blocks_replaced", "collapsed", "end_reason", "collapse_reason",
    "first_block", "arm_collisions_unchecked", "sim_seconds", "wall_seconds", "replay_file",
]


def perception_error(perceived: dict, truth: dict, unseen) -> dict:
    """Evaluation only: how far perception's tower poses were from the truth this turn (turns.csv columns).
    Height is the finger-height-critical one (0.875 mm margin, D-06); blocks nobody saw are counted, not scored."""
    seen = [b for b in perceived if b not in unseen]
    dz = [abs(perceived[b][2, 3] - truth[b][2, 3]) for b in seen]
    dp = [np.linalg.norm(perceived[b][:3, 3] - truth[b][:3, 3]) for b in seen]
    return dict(perception_unseen=len(unseen), perception_max_height_mm=round(max(dz, default=0.0) * 1000, 2),
                perception_max_pos_mm=round(max(dp, default=0.0) * 1000, 2))


def make_sim_world(cfg, tower, friction, record_replay):
    """Default world factory: the real Drake simulator.  (Tests pass their own stand-in.)"""
    from jenga.scene import Sim

    return Sim(cfg, tower, friction, record_replay)


def run_directory(cfg, mover, strategy: str, stamp: str = None) -> Path:
    """Where this run's files go: runs/ for real movers, runs_fake/ for the fake one."""
    root = Path(cfg.RUNS_DIR if mover.TAG == "real" else cfg.FAKE_RUNS_DIR)
    stamp = stamp or time.strftime("%Y%m%d_%H%M%S")
    return root / f"{strategy}_{stamp}"


def place_error(tower, target, poses, block):
    """Where the block ended against its target slot, with the slot's height taken from the MEASURED top of the
    layer below (mover.release_target, the height the place aims at, D-25), not the built height: the tower
    settles about 0.5 mm a layer, which is not a placement error.

    Inputs: tower (TowerState, before this turn's bookkeeping update), target ((layer, slot)), poses (block id ->
    4x4 world pose after the turn), block (the block that was moved).
    Returns: (4x4 slot pose used, distance m, (across, along, height) m in the slot's frame: along = the block's
    long axis, across = horizontal and perpendicular to it (what leaves room for a neighbour), height = up).
    """
    from jenga.mover import release_target  # pure numpy; the place aims at the same height
    X_T = release_target(tower, target[0], target[1], poses, tower.cfg, held=block)
    d = poses[block][:3, 3] - X_T[:3, 3]
    along, across, height = X_T[:3, :3].T @ d
    return X_T, float(np.linalg.norm(d)), (float(across), float(along), float(height))


def _append_csv(path: Path, columns, row: dict) -> None:
    new = not path.exists()
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns)
        if new:
            w.writeheader()
        w.writerow({c: row.get(c, "") for c in columns})


def run_trial(cfg, trial: int, mover, out_dir: Path, strategy: str = None, make_world=make_sim_world, log=print):
    """Run one trial and return its summary row (a dict).  Appends to turns.csv and trials.csv in out_dir."""
    strategy = cfg.STRATEGY if strategy is None else strategy
    seed = cfg.BASE_SEED + trial
    first_pick_rng = np.random.default_rng(seed)
    strategy_rng = np.random.default_rng(seed + 1_000_000)
    select = strategies.get_strategy(strategy)
    method = methods.get_method(cfg.EXTRACTION_METHOD)
    detect = collapse.get_detector(cfg.COLLAPSE_DETECTOR)
    unchecked = not clearance.ARM_COLLISIONS_CHECKED

    if mover.banner():
        log(mover.banner())

    tower = TowerState(cfg)
    friction = geo.assign_friction(cfg, seed)  # HIDDEN truth: goes to the simulator only
    world = make_world(cfg, tower, friction, cfg.RECORD_REPLAY)
    wall_trial0, sim_trial0 = time.time(), world.time()
    world.hold(cfg.START_SETTLE_S)  # the tower stands first: the replay opens on the intact tower, and nothing moves at t = 0

    common = dict(
        trial=trial, seed=seed, strategy=strategy, extraction_method=cfg.EXTRACTION_METHOD, mover=mover.TAG,
        friction_mode=cfg.FRICTION_MODE, n_layers_start=cfg.N_LAYERS_START, block_scale=cfg.BLOCK_SCALE,
        contact_model=cfg.CONTACT_MODEL, arm_collisions_unchecked=unchecked,
    )
    replaced, failures, first_block = 0, 0, ""
    end_reason, collapse_reason, collapsed, turns = "max_turns", "", False, 0

    seen = {}  # perception: what observe() saw last turn (its starting guesses for this turn)
    for turn in range(cfg.MAX_TURNS):
        perception_cols = {}
        if cfg.PERCEPTION:  # the tower bookkeeping from the cameras (perceive.py); scored against truth for the log
            poses, seen, unseen = perceive.observe(world, tower, cfg, seen)
            tower.update_poses(poses)
            perception_cols = perception_error(poses, world.block_poses(), unseen)
        else:
            tower.update_poses(world.block_poses())
        legal = tower.pull_candidates()
        candidates = [b for b in legal if method.can_pull(tower, b, cfg)]
        slots = [s for s in tower.placement_slots() if method.can_place(tower, s[0], s[1], cfg)]
        if not candidates:
            end_reason = "no_candidates"
            break
        if not slots:
            end_reason = "no_placement_slot"
            break

        if turn == 0:
            block = int(first_pick_rng.choice(np.array(candidates)))  # course rule: first block is random
            if cfg.DIAG_FIRST_BLOCK is not None:  # development only (R-03); drawn above anyway so the stream is unchanged
                if cfg.DIAG_FIRST_BLOCK not in candidates:
                    raise ValueError(f"DIAG_FIRST_BLOCK {cfg.DIAG_FIRST_BLOCK} is not a legal pull: {candidates}")
                block = int(cfg.DIAG_FIRST_BLOCK)
                log(f"[dev] first block forced to {block} (DIAG_FIRST_BLOCK): not a random pick, never for results")
            first_block = block
        else:
            block = int(select(tower, candidates, cfg, strategy_rng))
        if block not in candidates:
            raise ValueError(f"strategy {strategy!r} returned block {block}, which is not in the candidates {candidates}")

        target = slots[0]
        rec = tower.blocks[block]
        layer, slot = rec.layer, rec.slot
        before = world.block_poses()
        wall0, sim0 = time.time(), world.time()
        mover_error, info = "", None
        try:
            info = mover.do_turn(world, tower, block, target, cfg)
        except MotionFailed as e:  # a motion could not be done: logged as a failed turn
            mover_error = str(e)
        dlog = getattr(mover, "last_diag", None)  # development log (DIAG_LOG_LEVEL > 0), written even after a failure
        if dlog is not None:
            ids = dict(trial=trial, turn=turn, block_id=block)
            for r in dlog.rows:
                _append_csv(out_dir / "steps.csv", list(ids) + dlog.columns, dict(ids, **r))
            for r in dlog.block_rows:
                _append_csv(out_dir / "step_blocks.csv", list(ids) + diaglog.BLOCK_COLUMNS, dict(ids, **r))
        after = world.block_poses()
        others = max(float(np.linalg.norm(after[b][:3, 3] - before[b][:3, 3])) for b in before if b != block)
        X_slot, place_err, (p_across, p_along, p_height) = place_error(tower, target, after, block)
        log(f"   [check] other blocks moved at most {others * 1000:.2f} mm; placed block {place_err * 1000:.1f} mm from its slot "
            f"(across {p_across * 1000:.1f}, along {p_along * 1000:.1f}, height {p_height * 1000:.1f} mm; height against the "
            f"measured top of the layer below)")
        turns += 1

        outcome = outcomes.classify_turn(before[block], after[block], X_slot, cfg)  # slot at the measured height
        # the tower bookkeeping follows what really happened (with perception on, its poses come from the cameras at
        # the next turn's start instead: copying truth in here would start the next ICP at the right answer)
        if not cfg.PERCEPTION:
            tower.update_poses(after)
        if outcome == "replaced":
            tower.place(block, *target)
        elif outcome == "dropped":
            tower.mark_loose(block)
        verdict = detect(collapse.Truth(before, after, tower, block), cfg)
        if verdict.stop:
            outcome = "collapse"

        row = dict(common, turn=turn, block_id=block, layer=layer, slot=slot, target_layer=target[0],
                   target_slot=target[1], station_pull=getattr(info, "station_pull", ""),
                   station_place=getattr(info, "station_place", ""), outcome=outcome, collapse=verdict.stop,
                   collapse_reason=verdict.reason, mover_error=mover_error, sim_seconds=round(world.time() - sim0, 2),
                   place_across_mm=round(p_across * 1000, 2), place_along_mm=round(p_along * 1000, 2),
                   place_height_mm=round(p_height * 1000, 2),
                   wall_seconds=round(time.time() - wall0, 1), **perception_cols)
        _append_csv(out_dir / "turns.csv", TURN_COLUMNS, row)
        log(f"[trial {trial} turn {turn}] block {block} (layer {layer}, slot {slot}) -> {outcome}"
            + (f"  ({verdict.reason})" if verdict.stop else "") + (f"  [{mover_error}]" if mover_error else ""))

        if verdict.stop:
            collapsed, collapse_reason, end_reason = True, verdict.reason, "collapse"
            break
        if outcome == "replaced":
            replaced += 1
            failures = 0
        else:
            failures += 1
            if failures >= cfg.MAX_CONSECUTIVE_FAILURES:
                end_reason = "failures"
                break

    replay_file = ""
    if cfg.RECORD_REPLAY:
        path = out_dir / f"trial_{trial:02d}_replay.html"
        replay_file = str(path) if world.save_replay(path) else ""
    if cfg.ENABLE_WRIST_FORCE_LOG:
        world.save_force_log(out_dir / f"trial_{trial:02d}_wrist_force.csv")
    sim_seconds, wall_seconds = world.time() - sim_trial0, time.time() - wall_trial0
    summary = dict(common, turns_attempted=turns, blocks_replaced=replaced, collapsed=collapsed,
                   end_reason=end_reason, collapse_reason=collapse_reason, first_block=first_block,
                   sim_seconds=round(sim_seconds, 1), wall_seconds=round(wall_seconds, 1), replay_file=replay_file)
    _append_csv(out_dir / "trials.csv", TRIAL_COLUMNS, summary)
    if cfg.PRINT_SIM_SPEED and wall_seconds > 1.0 and sim_seconds > 0:
        log(f"[trial {trial}] {sim_seconds:.1f} s simulated in {wall_seconds:.1f} s wall ({sim_seconds / wall_seconds:.2f}x real time)")
    log(f"[trial {trial}] done: {replaced} block(s) replaced, ended by {end_reason}")
    return summary
