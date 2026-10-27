# Setup

This project uses its **own virtual environment**, separate from the problem sets'. It assumes you already did the class setup
(WSL with Ubuntu, `git`, `uv`). If you haven't, do that first: course README §3.1,
https://github.com/tlpmit/6.4210-2-fall26-handouts/blob/main/README.md

Inside the project folder (a path with no spaces, in the Linux filesystem, for example `~/robotics/jenga-iiwa`), run:

```
uv venv --python 3.13 && source .venv/bin/activate
uv pip install drake==1.57.0 "manipulation[letter]" scipy rtree
```

That is the whole setup. In every new terminal, activate it again with `source .venv/bin/activate` from the project folder.
Then run the checks in `README.md` (`python run_checks.py --quick` first).

## Watching a run
Runs are saved, not shown live (the live Meshcat page stays blank while a trial records; that is expected). Open the
`trial_XX_replay.html` files in the output folder: `explorer.exe .` opens the folder in Windows, then double-click the file.

## If something fails
`checks_report.txt` has the details; the summary table is at the top.
- "cannot import pydrake": the environment is not active (`source .venv/bin/activate`).
- `No module named X`: run `uv pip install X` in the active environment and tell the team, so this file can be corrected.
- `returncode == 512` during Drake's first model download: a space or special character in the path. Put the venv in your home
  directory instead (`uv venv --python 3.13 ~/venv-jenga`, install with `uv pip install --python ~/venv-jenga ...`); see the course README §3.3.
- the scene or the rolling base fails: set `BASE_MODE = "fixed"` in `config.py`, run the checks again, and send `checks_report.txt`.
