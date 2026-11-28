#!/bin/bash
# The final run (docs/FINAL_RUN.md): the conditions in cluster/final_conditions.py on trials 500-599, one Slurm
# array per condition, replays off.  Run from the frozen worktree (it refuses uncommitted changes):
#   bash cluster/final_run.sh <out_root> A B C D        then, when those are done,   ... E F G H
# Rerun crashed games of one condition:  TRIALS=503,517 bash cluster/final_run.sh <out_root> C
set -e
OUT=${1:?usage: bash cluster/final_run.sh <out_root> <condition letters ...>}
shift
[ $# -gt 0 ] || { echo "name the conditions to submit, e.g. A B C D" >&2; exit 1; }
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "uncommitted changes: the final run is submitted from a frozen worktree only" >&2; exit 1
fi
mkdir -p "$OUT" logs
while read -r L STRAT SETS; do
  for want in "$@"; do
    [ "$want" = "$L" ] || continue
    mkdir -p "$OUT/$L"
    git log -1 --format='%H %s' > "$OUT/$L/code.txt"
    id=$(OUT_ROOT=$OUT/$L STRATEGY=$STRAT REPLAY=0 SET="$SETS" \
         sbatch --parsable -J final_$L -t 03:00:00 -a ${TRIALS:-500-599}%25 cluster/jenga_array.sbatch)
    echo "$L $STRAT job $id  ($SETS)"
  done
done < <(python3 cluster/final_conditions.py)
