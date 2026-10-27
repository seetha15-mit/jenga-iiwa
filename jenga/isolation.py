"""Run checks in separate Python processes, so a hard Drake abort becomes a FAIL line instead of killing the run.  Pure Python.

The worker process prints one line per finished check:   RESULT {json}
If it dies before finishing the list, the check it was running is recorded as FAILED (with the last lines of
output) and the remaining checks run in a fresh process.  If the FIRST check that needs the simulator crashes,
the other checks that need it are skipped (the scene did not build), not repeated.
"""

import collections
import json
import subprocess


def result_line(name, status, seconds, headline, detail) -> str:
    return "RESULT " + json.dumps(dict(name=name, status=status, seconds=seconds, headline=headline, detail=detail))


def run_isolated(names, make_cmd, needs_sim=frozenset(), echo=print):
    """Run `names` through subprocesses.  make_cmd(remaining_names) -> argv list for a worker.
    Returns a list of (name, status, seconds, headline, detail)."""
    results, remaining = [], list(names)
    while remaining:
        proc = subprocess.Popen(make_cmd(remaining), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        tail = collections.deque(maxlen=40)
        finished = []
        for line in proc.stdout:
            line = line.rstrip("\n")
            if line.startswith("RESULT "):
                r = json.loads(line[len("RESULT "):])
                results.append((r["name"], r["status"], r["seconds"], r["headline"], r["detail"]))
                finished.append(r["name"])
                echo(f"[{r['name']}] {r['status']}: {r['headline']}  ({r['seconds']:.0f} s)")
            else:
                tail.append(line)
                echo(line)
        code = proc.wait()
        proc.stdout.close()
        remaining = [n for n in remaining if n not in finished]
        if not remaining:
            break
        crashed = remaining.pop(0)
        headline = f"the Python process died (exit code {code}): probably a hard abort inside Drake"
        results.append((crashed, "FAIL", 0.0, headline, "last output before the crash:\n" + "\n".join(tail)))
        echo(f"[{crashed}] FAIL: {headline}")
        sim_ran_before = any(r[0] in needs_sim for r in results[:-1])
        if crashed in needs_sim and not sim_ran_before:
            for n in [n for n in remaining if n in needs_sim]:
                results.append((n, "SKIP", 0.0, "skipped: the first check that builds the scene crashed", ""))
                echo(f"[{n}] SKIP: the first check that builds the scene crashed")
            remaining = [n for n in remaining if n not in needs_sim]
    return results
