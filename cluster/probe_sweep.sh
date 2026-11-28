#!/bin/bash
# Probe threshold sweep (D-28): mixed method on varied-friction towers, development trials 0-99.
#   off             rule-based, probe off: the baseline, and the first rule-based run with varied friction
#   measure         rule-based, probe on, thresholds None: every block probed and taken, readings in probes.csv
#   current         rule-based, probe on at the config's hand-picked thresholds (pull 3.5 N, push 4.0 N)
#   random_off      random, probe off
#   random_current  random, probe on at 3.5 N / 4.0 N
#   fast_measure    rule-based, measure-only with the probe leg at the push/pull's own speed (PROBE_SECONDS 0.1 s:
#                   2 mm at 20 mm/s), since the 1 s probe leg (2 mm/s) dragged the layer above (trial 9: 39 mm vs 0.35)
# Run from a frozen worktree of the commit under test:  bash cluster/probe_sweep.sh <out_dir> [setting ...]
# (no settings named: all of them)
set -e
OUT=${1:?usage: bash cluster/probe_sweep.sh <out_dir> [setting ...]}
shift
mkdir -p "$OUT" logs
git log -1 --format='%H %s' > "$OUT/code.txt"
BASE="FRICTION_MODE='random' DIAG_LOG_LEVEL=1"
OFF="PROBE_ENABLED=False"
MEASURE="PROBE_ENABLED=True PROBE_MAX_PULL_N=None PROBE_MAX_PUSH_N=None"
CURRENT="PROBE_ENABLED=True PROBE_MAX_PULL_N=3.5 PROBE_MAX_PUSH_N=4.0"
sub() {  # name strategy settings
  OUT_ROOT=$OUT/$1 STRATEGY=$2 REPLAY=0 SET="$BASE $3" \
    sbatch --parsable -J probe_$1 -t 02:00:00 -a 0-99%25 cluster/jenga_array.sbatch
}
for s in ${@:-off measure current random_off random_current}; do
  case $s in
    off)            echo "$s $(sub $s rule_based "$OFF")" ;;
    measure)        echo "$s $(sub $s rule_based "$MEASURE")" ;;
    current)        echo "$s $(sub $s rule_based "$CURRENT")" ;;
    random_off)     echo "$s $(sub $s random "$OFF")" ;;
    random_current) echo "$s $(sub $s random "$CURRENT")" ;;
    fast_measure)   echo "$s $(sub $s rule_based "$MEASURE PROBE_SECONDS=0.1")" ;;
    *) echo "unknown setting $s" >&2; exit 1 ;;
  esac
done
