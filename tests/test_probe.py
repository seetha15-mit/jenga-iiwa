"""Probe before taking a block (jenga/probe.py, D-28): the force measure, the accept rule, the fallback, and the
harness's drop-and-ask-again loop.  No physics: a stand-in mover decides which blocks resist."""

import csv
import tempfile
import unittest
from pathlib import Path

from jenga import probe, strategies
from jenga.evaluation import harness
from tests import helpers
from tests.helpers import MockWorld, make_cfg


def read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


class TestProbeRules(unittest.TestCase):
    def test_resistance_is_the_largest_change_from_the_tare(self):
        tare = [0.0, -10.7, 0.0, 0.1, 0.2, 0.3]  # the hand's weight on G y (sensing.py)
        trace = [[0.0, -10.7, 0.0, 9, 9, 9],  # torque changes are ignored
                 [3.0, -6.7, 0.0, 0, 0, 0],  # change (3, 4, 0): 5 N
                 [1.0, -10.7, 0.0, 0, 0, 0]]
        self.assertAlmostEqual(probe.resistance(trace, tare), 5.0)
        self.assertEqual(probe.resistance([], tare), 0.0)

    def test_accept(self):
        self.assertTrue(probe.accept(99.0, make_cfg(PROBE_MAX_PULL_N=None), "pull"))  # measure only
        self.assertTrue(probe.accept(2.0, make_cfg(PROBE_MAX_PUSH_N=2.0), "push"))
        self.assertFalse(probe.accept(2.1, make_cfg(PROBE_MAX_PULL_N=2.0), "pull"))

    def test_threshold_by_step(self):
        cfg = make_cfg(PROBE_MAX_PULL_N=7.0, PROBE_MAX_PUSH_N=3.5)
        self.assertTrue(probe.accept(6.5, cfg, "pull"))
        self.assertFalse(probe.accept(6.5, cfg, "push"))
        self.assertTrue(probe.accept(99.0, make_cfg(PROBE_MAX_PULL_N=7.0, PROBE_MAX_PUSH_N=None), "push"))  # never
        self.assertEqual(probe.max_force(cfg, "push"), 3.5)

    def test_fallback_is_the_least_resisting_block(self):
        self.assertEqual(probe.fallback({4: 3.0, 7: 1.5, 2: 2.0}), 7)
        self.assertEqual(probe.fallback({4: 1.5, 2: 1.5}), 2)  # a tie goes to the lower id (repeatable)
        self.assertIsNone(probe.fallback({}))


class ProbeMover(helpers.RealLikeMover):
    """Probes by table lookup: resistance[block] (default 1 N).  Records (block, probe) for every call.
    shove: on a rejected probe, move another block this far in m (to test the collapse check), or None."""

    def __init__(self, resistance=None, shove=None):
        self.resistance, self.shove, self.calls, self.last_probe = resistance or {}, shove, [], None

    def do_turn(self, world, tower, block_id, target, cfg, probe=False):
        self.calls.append((block_id, probe))
        self.last_probe = None
        if probe:
            r = self.resistance.get(block_id, 1.0)
            ok = probe_accept(r, cfg, "pull")
            self.last_probe = dict(block_id=block_id, layer=tower.blocks[block_id].layer,
                                   slot=tower.blocks[block_id].slot, step="pull", station=0, resistance_n=r,
                                   samples=50, accepted=ok)
            if not ok:
                if self.shove:
                    other = next(b for b in tower.blocks if b != block_id)
                    world.poses[other] = world.poses[other].copy()
                    world.poses[other][0, 3] += self.shove
                raise probe_rejected(block_id, r)
        return super().do_turn(world, tower, block_id, target, cfg)


probe_accept, probe_rejected = probe.accept, probe.ProbeRejected  # do_turn's `probe` argument hides the module


class TestHarnessWithProbe(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        strategies.REGISTRY["lowest"] = "tests.helpers_lowest"
        self.cfg = make_cfg(RECORD_REPLAY=False, RUNS_DIR=str(self.root / "runs"),
                            FAKE_RUNS_DIR=str(self.root / "runs_fake"), MAX_TURNS=2, EXTRACTION_METHOD="end_pinch",
                            PROBE_ENABLED=True, PROBE_MAX_PULL_N=2.0, PROBE_MAX_PER_TURN=3)

    def tearDown(self):
        strategies.REGISTRY.pop("lowest", None)
        self.tmp.cleanup()

    def run_trial(self, mover, strategy="lowest", **over):
        cfg = make_cfg(**{**vars(self.cfg), **over})
        self.n_runs = getattr(self, "n_runs", 0) + 1  # each run its own folder: the CSVs are appended to
        out = harness.run_directory(cfg, mover, strategy, stamp=f"p{self.n_runs}")
        s = harness.run_trial(cfg, 0, mover, out, strategy=strategy, make_world=MockWorld, log=lambda *_: None)
        return s, out

    def test_a_resisting_block_is_dropped_and_the_strategy_asked_again(self):
        base = ProbeMover()
        self.run_trial(base)
        first, choice = base.calls[0][0], base.calls[1][0]  # turn 0: the random pick; turn 1: the strategy's choice
        m = ProbeMover(resistance={choice: 5.0})
        s, out = self.run_trial(m)
        self.assertEqual(m.calls[0][0], first)  # turn 0 is still the seed's random block
        self.assertEqual(m.calls[1], (choice, True))  # probed, resisted
        self.assertNotEqual(m.calls[2][0], choice)  # the strategy did not get it again
        self.assertTrue(m.calls[2][1])  # the next one was probed too
        row = read(out / "turns.csv")[1]
        self.assertEqual((row["probes_rejected"], row["block_id"], row["outcome"]), ("1", str(m.calls[2][0]), "replaced"))
        probes = read(out / "probes.csv")
        self.assertEqual([p["accepted"] for p in probes], ["True", "False", "True"])
        self.assertEqual((probes[1]["block_id"], probes[1]["turn"], probes[1]["max_force_n"]), (str(choice), "1", "2.0"))

    def test_after_the_cap_the_least_resisting_block_is_taken_without_a_probe(self):
        m = ProbeMover(resistance={b: 10.0 + b for b in range(30)})  # everything resists; low ids least
        s, out = self.run_trial(m, MAX_TURNS=1)
        calls = m.calls
        self.assertEqual(len(calls), 4)
        self.assertTrue(all(p for _, p in calls[:3]))
        self.assertEqual(calls[3], (min(b for b, _ in calls[:3]), False))
        self.assertEqual(read(out / "turns.csv")[0]["probes_rejected"], "3")
        self.assertEqual(len(read(out / "probes.csv")), 3)  # the unprobed take has no probe row
        self.assertEqual(s["blocks_replaced"], 1)

    def test_a_probe_that_knocks_the_tower_ends_the_trial(self):
        m = ProbeMover(resistance={b: 10.0 for b in range(30)}, shove=0.05)
        s, out = self.run_trial(m, MAX_TURNS=1)
        self.assertEqual(len(m.calls), 1)  # no second try on a knocked tower
        self.assertTrue(s["collapsed"])
        row = read(out / "turns.csv")[0]
        self.assertEqual(row["outcome"], "collapse")
        self.assertIn("resisted", row["mover_error"])

    def test_with_perception_a_rejection_reobserves_the_tower_instead_of_copying_truth(self):
        from unittest import mock
        from jenga import perceive
        calls = []

        def fake_observe(world, tower, cfg, last):
            calls.append(len(calls))
            poses = world.block_poses()
            for X in poses.values():
                X[0, 3] += 1e-4  # "perceived" = truth + 0.1 mm, so a copy of the truth would show
            return poses, {}, []

        m = ProbeMover(resistance={b: 10.0 for b in range(30)})  # everything resists
        with mock.patch.object(perceive, "observe", side_effect=fake_observe):
            s, out = self.run_trial(m, MAX_TURNS=1, PERCEPTION=True)
        self.assertEqual(len(calls), 1 + 3)  # the turn's start, then once after each of the 3 rejections
        self.assertEqual(read(out / "turns.csv")[0]["probes_rejected"], "3")

    def test_measure_only_never_rejects(self):
        m = ProbeMover(resistance={b: 50.0 for b in range(30)})
        s, out = self.run_trial(m, PROBE_MAX_PULL_N=None)
        self.assertEqual([p for _, p in m.calls], [True] * len(m.calls))
        self.assertEqual(len(m.calls), s["turns_attempted"])
        self.assertTrue(all(p["accepted"] == "True" for p in read(out / "probes.csv")))

    def test_probe_off_calls_the_mover_as_before(self):
        s, out = self.run_trial(helpers.RealLikeMover(), PROBE_ENABLED=False)  # its do_turn has no probe argument
        self.assertFalse((out / "probes.csv").exists())
        self.assertTrue(all(r["probes_rejected"] == "0" for r in read(out / "turns.csv")))


if __name__ == "__main__":
    unittest.main()
