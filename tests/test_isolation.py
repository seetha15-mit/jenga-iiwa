import sys
import unittest

from jenga import isolation

# A stand-in worker: prints a RESULT line per name; a name starting with "crash" aborts the whole process.
WORKER = r"""
import os, sys, json
sys.path.insert(0, '.')
from jenga import isolation
for n in sys.argv[1].split(','):
    print('building something ...', flush=True)
    if n.startswith('crash'):
        sys.stderr.write('Failure at fake.cc:42: condition failed\n'); sys.stderr.flush()
        os.abort()
    print(isolation.result_line(n, 'PASS', 0.1, 'ok ' + n, 'detail'), flush=True)
"""


def cmd(remaining):
    return [sys.executable, "-c", WORKER, ",".join(remaining)]


class TestIsolation(unittest.TestCase):
    def run_it(self, names, needs_sim=frozenset()):
        out = []
        res = isolation.run_isolated(names, cmd, needs_sim, echo=out.append)
        return {r[0]: r for r in res}, res, out

    def test_all_pass(self):
        by, res, _ = self.run_it(["a", "b"])
        self.assertEqual([r[1] for r in res], ["PASS", "PASS"])

    def test_a_crash_becomes_a_fail_with_the_last_output_and_the_rest_still_run(self):
        by, res, _ = self.run_it(["a", "crash1", "c"])
        self.assertEqual([r[0] for r in res], ["a", "crash1", "c"])
        self.assertEqual(by["a"][1], "PASS")
        self.assertEqual(by["crash1"][1], "FAIL")
        self.assertIn("Failure at fake.cc:42", by["crash1"][4])
        self.assertEqual(by["c"][1], "PASS")

    def test_if_the_scene_never_built_the_other_sim_checks_are_skipped(self):
        by, _, _ = self.run_it(["reach", "crash_scene", "x", "y"], needs_sim={"crash_scene", "x"})
        self.assertEqual(by["crash_scene"][1], "FAIL")
        self.assertEqual(by["x"][1], "SKIP")
        self.assertEqual(by["y"][1], "PASS")  # does not need the simulator

    def test_a_later_crash_does_not_skip_the_others(self):
        by, _, _ = self.run_it(["scene", "crash_exec", "base"], needs_sim={"scene", "crash_exec", "base"})
        self.assertEqual(by["scene"][1], "PASS")
        self.assertEqual(by["crash_exec"][1], "FAIL")
        self.assertEqual(by["base"][1], "PASS")


if __name__ == "__main__":
    unittest.main()
