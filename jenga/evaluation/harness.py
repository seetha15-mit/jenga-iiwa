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

from jenga import clearance, geometry as geo, methods, strategies
from jenga.errors import MotionFailed
from jenga.evaluation import collapse, outcomes
from jenga.tower import TowerState

TURN_COLUMNS = [
    "trial", "seed", "strategy", "extraction_method", "mover", "friction_mode", "n_layers_start", "block_scale",
    "contact_model", "turn", "block_id", "layer", "slot", "target_layer", "target_slot", "station_pull",
    "station_place", "outcome", "collapse", "collapse_reason", "mover_error", "arm_collisions_unchecked",
    "sim_seconds", "wall_seconds",
]
TRIAL_COLUMNS = [
    "trial", "seed", "strategy", "extraction_method", "mover", "friction_mode", "n_layers_start", "block_scale",
    "contact_model", "turns_attempted", "blocks_replaced", "collapsed", "end_reason", "collapse_reason",
    "first_block", "arm_collisions_unchecked", "sim_seconds", "wall_seconds", "replay_file",
]


def make_sim_world(cfg, tower, friction, record_replay):
    """Default world factory: the real Drake simulator.  (Tests pass their own stand-in.)"""
    from jenga.scene import Sim

    return Sim(cfg, tower, friction, record_replay)


def run_directory(cfg, mover, strategy: str, stamp: str = None) -> Path:
    """Where this run's files go: runs/ for real movers, runs_fake/ for the fake one."""
    root = Path(cfg.RUNS_DIR if mover.TAG == "real" else cfg.FAKE_RUNS_DIR)
    stamp = stamp or time.strftime("%Y%m%d_%H%M%S")
    return root / f"{strategy}_{stamp}"


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

    for turn in range(cfg.MAX_TURNS):
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
        after = world.block_poses()
        turns += 1

        outcome = outcomes.classify_turn(before[block], after[block], tower.target_pose(*target), cfg)
        # the tower bookkeeping follows what really happened
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
                   wall_seconds=round(time.time() - wall0, 1))
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
