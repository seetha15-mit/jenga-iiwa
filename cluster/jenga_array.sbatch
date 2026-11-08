#!/bin/bash
#SBATCH -J jenga
#SBATCH -p mit_normal
#SBATCH -t 03:00:00
#SBATCH -c 1
#SBATCH --mem=6G
#SBATCH -a 0-9
#SBATCH -o logs/%x-%A_%a.out

module load apptainer/1.4.2
export DRAKE_NUM_THREADS=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1

SCRATCH_ROOT=$HOME/orcd/scratch
apptainer exec -B $SCRATCH_ROOT --pwd $SLURM_SUBMIT_DIR jenga.sif \
  python run_one_trial.py --strategy ${STRATEGY:-random} --trial $SLURM_ARRAY_TASK_ID \
  --mover real --out-root ${OUT_ROOT:-$SCRATCH_ROOT/jenga_runs} ${TURNS:+--turns $TURNS}