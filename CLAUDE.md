# Ground rules for AI tools working in this repo (Claude Code or any other)

1. **Read `docs/DECISIONS.md` first.** It is the current truth. If a change touches a decision, edit that entry IN PLACE (no history, no "superseded"; git keeps history) and keep the file under two pages.
2. **Never write the team-owned parts unless asked by name:** `plan_pull` and `plan_place` in `jenga/methods/end_pinch.py`, the strategies in `jenga/strategies/` (except `random_baseline.py`), the advanced collapse detector, `jenga/clearance.py`, and anything in `jenga/sensing.py` beyond notes. They are marked `CODE TO BE WRITTEN`. The course requires the team to own and justify these choices.
3. **Label every number** in config and docs as `[user] [course] [measured] [derived] [estimate]`. Do not present a guess as a result. Say "I'm not sure" when that is true. Drake code written from memory is untested: say so, and say how to check it (`python run_checks.py`).
4. **Never change a value in `config.py` without the owner's say-so**, and never move a setting out of it.
5. **Hidden from strategies:** per-block friction and exact contact forces. Strategies never import from `jenga.evaluation` or the simulator's truth. Fake-mover output stays in `runs_fake/` and never enters results.
6. **Keep it simple.** Prefer fewer moving parts; tell the owner what an addition costs. Do not add features that were not asked for. Plain functions, pset style, docstrings that say what and why without padding.
7. **Run `python run_checks.py` (or `--quick` without Drake) before saying something works.** Add a test with every pure-Python change (`tests/`).
8. **Ask before starting a new stage or restructuring.** If an answer contradicts a decision, say so before acting.
9. AI use must be citable: say in the commit message what the tool did.
