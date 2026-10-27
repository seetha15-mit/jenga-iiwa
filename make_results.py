"""Build the results table from real runs.   python make_results.py [runs_folder]

Refuses to run on anything from the fake mover (see jenga/evaluation/results.py).
"""

import sys

import config
from jenga.evaluation import results

if __name__ == "__main__":
    folder = sys.argv[1] if len(sys.argv) > 1 else config.RUNS_DIR
    try:
        sys.exit(results.main(folder, config))
    except results.FakeRunsError as e:
        print(f"REFUSED: {e}")
        sys.exit(2)
