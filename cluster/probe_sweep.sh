#!/bin/bash
# Probe threshold sweep (D-28): rule-based mixed on varied-friction towers, development trials 0-99.
#   off      probe off: the baseline, and the first rule-based run with varied friction
#   measure  probe on, thresholds None: every block probed and taken, readings in probes.csv
#   current  probe on at the config's hand-picked thresholds (pull 3.5 N, push 4.0 N)
# Run from a frozen worktree of the commit under test:  bash cluster/probe_sweep.sh <out_dir>
set -e
OUT=${1:?usage: bash cluster/probe_sweep.sh <out_dir>}
mkdir -p "$OUT" logs
git log -1 --format='%H %s' > "$OUT/code.txt"
BASE="FRICTION_MODE='random' DIAG_LOG_LEVEL=1"
sub() {
  OUT_ROOT=$OUT/$1 STRATEGY=rule_based REPLAY=0 SET="$BASE $2" \
    sbatch --parsable -J probe_$1 -t 02:00:00 -a 0-99%25 cluster/jenga_array.sbatch
}
echo "off     $(sub off     'PROBE_ENABLED=False')"
echo "measure $(sub measure 'PROBE_ENABLED=True PROBE_MAX_PULL_N=None PROBE_MAX_PUSH_N=None')"
echo "current $(sub current 'PROBE_ENABLED=True PROBE_MAX_PULL_N=3.5 PROBE_MAX_PUSH_N=4.0')"
