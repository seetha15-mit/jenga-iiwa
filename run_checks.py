"""ONE COMMAND THAT CHECKS EVERYTHING.     python run_checks.py

It runs, in order:
    config          every setting has a source tag; settings that cannot work are flagged
    unit tests      the pure-Python tests in tests/ (no Drake needed)
    reachability    every layer (and the placement level) reachable with a level hand?  (arm only)
    scene + static  the Drake scene builds and the tower stands still; simulation speed
    gripper         the pad gap read from the gripper model; the block fits
    robot frame     the arm-only plant and the full scene agree about where the hand is
    wrist force     raw force at rest is about the hand's weight
    executor        four motions in ONE persistent simulator reach their targets
    base            the rolling base reaches stations without disturbing the tower
    fake trial      a 2-turn fake-mover trial writes tagged files and a replay; the results guard refuses it

and writes everything to  checks_report.txt.   Paste that file if you need help.

    python run_checks.py --quick            only config and unit tests (no Drake, about 2 seconds)
    python run_checks.py --compare-contact  also compare point and hydroelastic contact (slow)

Every Drake check was written without being able to run Drake.  A FAIL there is information, not blame:
the report says which call or number was off.
"""

import argparse
import importlib
import io
import json
import platform
import sys
import time
import traceback
import unittest
from pathlib import Path
from types import SimpleNamespace

import config as _config
from jenga import config_check, isolation

REPORT = Path("checks_report.txt")


def _cfg():
    return SimpleNamespace(**{k: v for k, v in vars(_config).items() if k.isupper()})


def check_config(cfg, shared):
    warnings = config_check.check_tags(Path("config.py")) + config_check.check_values(cfg)
    detail = config_check.describe(cfg) + ("\n\n" + "\n".join(warnings) if warnings else "")
    real = [w for w in warnings if "MOVER = 'fake'" not in w]
    if real:
        return "WARN", f"{len(real)} warning(s)", detail
    return "PASS", "no warnings" + ("  (note: MOVER is 'fake')" if warnings else ""), detail


def check_unit_tests(cfg, shared):
    stream = io.StringIO()
    suite = unittest.defaultTestLoader.discover("tests")
    result = unittest.TextTestRunner(stream=stream, verbosity=1).run(suite)
    text = stream.getvalue()
    status = "PASS" if result.wasSuccessful() else "FAIL"
    return status, f"{result.testsRun} tests, {len(result.failures)} failed, {len(result.errors)} errors", text


def drake_available():
    for mod in ("pydrake", "manipulation"):
        try:
            importlib.import_module(mod)
        except Exception as e:  # noqa: BLE001
            return False, f"cannot import {mod}: {e}"
    return True, ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="only config and unit tests (no Drake)")
    ap.add_argument("--compare-contact", action="store_true", help="also compare point vs hydroelastic contact")
    ap.add_argument("--worker", help=argparse.SUPPRESS)
    args = ap.parse_args()
    cfg = _cfg()
    shared = {}

    checks = [("config", check_config), ("unit tests", check_unit_tests)]
    skip_reason = "--quick" if args.quick else None
    if not args.quick:
        ok, why = drake_available()
        if not ok:
            skip_reason = why
    from jenga import drake_checks as dc

    drake_list = [
        ("reachability", dc.check_reachability), ("scene + static tower", dc.check_scene_and_static),
        ("gripper numbers", dc.check_gripper), ("robot frame", dc.check_robot_frame),
        ("wrist force at rest", dc.check_wrist_force), ("executor", dc.check_executor),
        ("rolling base", dc.check_base), ("fake trial + guard", dc.check_fake_trial),
    ]
    if args.compare_contact:
        drake_list.append(("contact model comparison", dc.check_compare_contact))

    drake_fns = dict(drake_list)
    if args.worker:  # internal: run these Drake checks in THIS process and report each result as a RESULT line
        shared = {}
        for name in args.worker.split(","):
            r = _run(name, drake_fns[name], cfg, shared, quiet=True)
            print(isolation.result_line(*r), flush=True)
        return

    results = []
    for name, fn in checks:
        results.append(_run(name, fn, cfg, shared))
    if skip_reason:
        results += [(name, "SKIP", 0.0, f"skipped ({skip_reason})", "") for name, _ in drake_list]
    else:
        flags = ["--compare-contact"] if args.compare_contact else []
        needs_sim = frozenset(n for n, _ in drake_list if n != "reachability")
        results += isolation.run_isolated(
            [n for n, _ in drake_list],
            lambda rem: [sys.executable, "run_checks.py", "--worker", ",".join(rem)] + flags,
            needs_sim=needs_sim,
        )

    _write_report(results, args)
    failed = [r for r in results if r[1] == "FAIL"]
    print(f"\nFull report: {REPORT.resolve()}")
    sys.exit(1 if failed else 0)


def _run(name, fn, cfg, shared, quiet=False):
    print(f"[{name}] running ...", flush=True)
    t0 = time.time()
    try:
        status, headline, detail = fn(cfg, shared)
    except NotImplementedError as e:
        status, headline, detail = "FAIL", f"not implemented: {e}", traceback.format_exc()
    except Exception as e:  # noqa: BLE001
        status, headline, detail = "FAIL", f"{type(e).__name__}: {str(e).splitlines()[0] if str(e) else ''}", traceback.format_exc()
    dt = time.time() - t0
    if not quiet:
        print(f"[{name}] {status}: {headline}  ({dt:.0f} s)", flush=True)
    return (name, status, dt, headline, detail)


def _versions():
    out = [f"date: {time.strftime('%Y-%m-%dT%H:%M:%S')}", f"python: {sys.version.split()[0]} ({sys.executable})",
           f"platform: {platform.platform()}"]
    for mod in ("pydrake", "manipulation", "numpy", "scipy"):
        try:
            m = importlib.import_module(mod)
            v = getattr(m, "__version__", None)
            if v is None:
                import importlib.metadata as md

                v = md.version({"pydrake": "drake"}.get(mod, mod))
            out.append(f"{mod}: {v}")
        except Exception:  # noqa: BLE001
            out.append(f"{mod}: not available")
    return out


def _write_report(results, args):
    summary = ["SUMMARY", f"{'check':26s} {'status':7s} {'seconds':>8s}  headline", "-" * 100]
    for name, status, dt, headline, _ in results:
        summary.append(f"{name:26s} {status:7s} {dt:8.0f}  {headline}")
    lines = ["CHECKS", ""] + _versions() + [""] + summary + [""]
    for name, status, dt, headline, detail in results:
        lines += ["=" * 100, f"{name}   [{status}, {dt:.0f} s]", "=" * 100, headline, "", detail, ""]
    REPORT.write_text("\n".join(lines))
    print("\n" + "\n".join(summary))


if __name__ == "__main__":
    main()
