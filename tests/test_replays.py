"""Replay selection rules (cluster/replays.py, docs/FINAL_RUN.md Replays)."""
import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "cluster"))
import replays  # noqa: E402


def write_condition(root, letter, strategy, blocks):
    for t, b in blocks.items():
        d = Path(root) / letter / f"{strategy}_trial{t:02d}"
        d.mkdir(parents=True)
        with open(d / "trials.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["trial", "seed", "blocks_replaced", "end_reason", "collapsed", "replay_file"])
            w.writeheader()
            w.writerow(dict(trial=t, seed=1000 + t, blocks_replaced=b, end_reason="collapse", collapsed=True, replay_file=""))


class TestPick(unittest.TestCase):
    def test_best_worst_typical_ties_go_to_the_lowest_trial(self):
        b = {5: 7, 3: 7, 4: 1, 9: 1, 1: 4, 2: 5}  # median 4.5: 4 and 5 are equally close -> trial 1 (4) or 2 (5)
        self.assertEqual(replays.pick_condition(b), [("best", 3), ("worst", 4), ("typical", 1)])

    def test_pairs_gain_and_loss(self):
        first, second = {0: 5, 1: 5, 2: 5, 3: 5}, {0: 8, 1: 2, 2: 8, 3: 5}
        self.assertEqual([t for _, t in replays.pick_pairs(first, second)], [0, 1])
        self.assertTrue(replays.pick_pairs(first, second)[0][0].startswith("gained most (+3)"))

    def test_selection_reruns_each_game_once_with_all_its_reasons(self):
        with tempfile.TemporaryDirectory() as tmp:
            write_condition(tmp, "A", "random", {0: 1, 1: 3, 2: 2})
            write_condition(tmp, "B", "rule_based", {0: 6, 1: 3, 2: 2})
            sel = replays.selection(Path(tmp))
            games = [(L, t) for L, t, _ in sel]
            self.assertEqual(len(games), len(set(games)))
            reasons = dict(((L, t), r) for L, t, r in sel)
            self.assertIn("A worst", reasons[("A", 0)])
            self.assertIn("M1 (B vs A) gained most (+5)", reasons[("A", 0)])
            self.assertIn("M1 (B vs A) gained most (+5)", reasons[("B", 0)])

    def test_submit_skips_games_already_replayed(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "replays").mkdir()
            with open(Path(tmp) / "replays" / "selection.csv", "w", newline="") as f:
                f.write("condition,trial,reasons\nA,500,x\nA,501,x\nE,500,x\n")
            write_condition(Path(tmp) / "replays", "A", "random", {500: 3})
            self.assertEqual(replays.to_submit(Path(tmp)), {"A": [501], "E": [500]})

    def test_replay_path_is_relative_to_where_the_run_is_now(self):
        rerun = {"replay_file": "/old/place/replays/H/rule_based_trial512/trial_512_replay.html"}
        self.assertEqual(replays.replay_path(Path("/new"), "H", 512, rerun),
                         "replays/H/rule_based_trial512/trial_512_replay.html")
        self.assertEqual(replays.replay_path(Path("/new"), "H", 512, None), "")

    def test_command_carries_the_conditions_settings(self):
        cmd = replays.command("H", 512, "/out")
        for s in ("--strategy rule_based", "--trial 512", "--replay", "FRICTION_MODE='random'", "PROBE_EDGE_PUSH=True"):
            self.assertIn(s, cmd)


if __name__ == "__main__":
    unittest.main()
