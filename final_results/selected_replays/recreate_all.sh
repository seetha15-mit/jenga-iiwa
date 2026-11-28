#!/bin/bash
# Remake every replay in replays.csv on ORCD (one Slurm job each).  Run from the repo root,
# checked out at commit 7345e58, with jenga.sif next to it.  See README.txt.
set -e
mkdir -p logs
# videos/A_random_trial500.html: typical random game (blocks closest to the median)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/A STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 500 cluster/jenga_array.sbatch
# videos/A_random_trial504.html: worst random game (fewest blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/A STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 504 cluster/jenga_array.sbatch
# videos/A_random_trial512.html: best random game (most blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/A STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 512 cluster/jenga_array.sbatch
# videos/A_random_trial517.html: M1 pair: the game where rule-based (B) gained the most against random (A), +7 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/A STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 517 cluster/jenga_array.sbatch
# videos/A_random_trial547.html: M1 pair: the game where rule-based (B) lost the most against random (A), -2 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/A STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 547 cluster/jenga_array.sbatch
# videos/B_rule_based_trial500.html: typical rule-based game (blocks closest to the median)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/B STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 500 cluster/jenga_array.sbatch
# videos/B_rule_based_trial512.html: best rule-based game (most blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/B STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 512 cluster/jenga_array.sbatch
# videos/B_rule_based_trial517.html: M1 pair: the game where rule-based (B) gained the most against random (A), +7 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/B STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 517 cluster/jenga_array.sbatch
# videos/B_rule_based_trial522.html: M2 pair: the game where rule-based + perception (C) gained the most against rule-based (B), +4 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/B STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 522 cluster/jenga_array.sbatch
# videos/B_rule_based_trial545.html: worst rule-based game (fewest blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/B STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 545 cluster/jenga_array.sbatch
# videos/B_rule_based_trial547.html: M1 pair: the game where rule-based (B) lost the most against random (A), -2 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/B STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 547 cluster/jenga_array.sbatch
# videos/B_rule_based_trial581.html: M2 pair: the game where rule-based + perception (C) lost the most against rule-based (B), -3 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/B STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1" sbatch -t 03:00:00 -a 581 cluster/jenga_array.sbatch
# videos/C_rule_based_trial500.html: typical rule-based + perception game (blocks closest to the median)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/C STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 PERCEPTION=True" sbatch -t 03:00:00 -a 500 cluster/jenga_array.sbatch
# videos/C_rule_based_trial502.html: best rule-based + perception game (most blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/C STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 PERCEPTION=True" sbatch -t 03:00:00 -a 502 cluster/jenga_array.sbatch
# videos/C_rule_based_trial522.html: M2 pair: the game where rule-based + perception (C) gained the most against rule-based (B), +4 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/C STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 PERCEPTION=True" sbatch -t 03:00:00 -a 522 cluster/jenga_array.sbatch
# videos/C_rule_based_trial545.html: worst rule-based + perception game (fewest blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/C STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 PERCEPTION=True" sbatch -t 03:00:00 -a 545 cluster/jenga_array.sbatch
# videos/C_rule_based_trial581.html: M2 pair: the game where rule-based + perception (C) lost the most against rule-based (B), -3 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/C STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 PERCEPTION=True" sbatch -t 03:00:00 -a 581 cluster/jenga_array.sbatch
# videos/D_rule_based_trial500.html: best rule-based, varied friction game (most blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/D STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 500 cluster/jenga_array.sbatch
# videos/D_rule_based_trial501.html: typical rule-based, varied friction game (blocks closest to the median)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/D STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 501 cluster/jenga_array.sbatch
# videos/D_rule_based_trial544.html: M3 pair: the game where rule-based + probe (E) lost the most against rule-based, varied friction (D), -3 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/D STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 544 cluster/jenga_array.sbatch
# videos/D_rule_based_trial545.html: S1 pair: the game where rule-based + probe on all blocks (H) gained the most against rule-based, varied friction (D), +6 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/D STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 545 cluster/jenga_array.sbatch
# videos/D_rule_based_trial553.html: M3 pair: the game where rule-based + probe (E) gained the most against rule-based, varied friction (D), +4 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/D STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 553 cluster/jenga_array.sbatch
# videos/D_rule_based_trial570.html: worst rule-based, varied friction game (fewest blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/D STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 570 cluster/jenga_array.sbatch
# videos/D_rule_based_trial583.html: S1 pair: the game where rule-based + probe on all blocks (H) lost the most against rule-based, varied friction (D), -5 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/D STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 583 cluster/jenga_array.sbatch
# videos/E_rule_based_trial500.html: best rule-based + probe game (most blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/E STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 500 cluster/jenga_array.sbatch
# videos/E_rule_based_trial501.html: typical rule-based + probe game (blocks closest to the median)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/E STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 501 cluster/jenga_array.sbatch
# videos/E_rule_based_trial544.html: M3 pair: the game where rule-based + probe (E) lost the most against rule-based, varied friction (D), -3 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/E STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 544 cluster/jenga_array.sbatch
# videos/E_rule_based_trial553.html: M3 pair: the game where rule-based + probe (E) gained the most against rule-based, varied friction (D), +4 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/E STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 553 cluster/jenga_array.sbatch
# videos/E_rule_based_trial570.html: worst rule-based + probe game (fewest blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/E STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 570 cluster/jenga_array.sbatch
# videos/F_random_trial500.html: typical random, varied friction game (blocks closest to the median)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/F STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 500 cluster/jenga_array.sbatch
# videos/F_random_trial552.html: worst random, varied friction game (fewest blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/F STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 552 cluster/jenga_array.sbatch
# videos/F_random_trial558.html: best random, varied friction game (most blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/F STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 558 cluster/jenga_array.sbatch
# videos/F_random_trial589.html: M4 pair: the game where random + probe (G) gained the most against random, varied friction (F), +4 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/F STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 589 cluster/jenga_array.sbatch
# videos/F_random_trial591.html: M4 pair: the game where random + probe (G) lost the most against random, varied friction (F), -4 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/F STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random'" sbatch -t 03:00:00 -a 591 cluster/jenga_array.sbatch
# videos/G_random_trial500.html: typical random + probe game (blocks closest to the median)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/G STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 500 cluster/jenga_array.sbatch
# videos/G_random_trial520.html: worst random + probe game (fewest blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/G STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 520 cluster/jenga_array.sbatch
# videos/G_random_trial558.html: best random + probe game (most blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/G STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 558 cluster/jenga_array.sbatch
# videos/G_random_trial589.html: M4 pair: the game where random + probe (G) gained the most against random, varied friction (F), +4 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/G STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 589 cluster/jenga_array.sbatch
# videos/G_random_trial591.html: M4 pair: the game where random + probe (G) lost the most against random, varied friction (F), -4 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/G STRATEGY=random REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True" sbatch -t 03:00:00 -a 591 cluster/jenga_array.sbatch
# videos/H_rule_based_trial506.html: typical rule-based + probe on all blocks game (blocks closest to the median)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/H STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True PROBE_END_PULLS=True PROBE_EDGE_PUSH=True" sbatch -t 03:00:00 -a 506 cluster/jenga_array.sbatch
# videos/H_rule_based_trial545.html: S1 pair: the game where rule-based + probe on all blocks (H) gained the most against rule-based, varied friction (D), +6 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/H STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True PROBE_END_PULLS=True PROBE_EDGE_PUSH=True" sbatch -t 03:00:00 -a 545 cluster/jenga_array.sbatch
# videos/H_rule_based_trial560.html: best rule-based + probe on all blocks game (most blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/H STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True PROBE_END_PULLS=True PROBE_EDGE_PUSH=True" sbatch -t 03:00:00 -a 560 cluster/jenga_array.sbatch
# videos/H_rule_based_trial570.html: worst rule-based + probe on all blocks game (fewest blocks)
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/H STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True PROBE_END_PULLS=True PROBE_EDGE_PUSH=True" sbatch -t 03:00:00 -a 570 cluster/jenga_array.sbatch
# videos/H_rule_based_trial583.html: S1 pair: the game where rule-based + probe on all blocks (H) lost the most against rule-based, varied friction (D), -5 blocks
OUT_ROOT=$HOME/orcd/scratch/jenga_replays/H STRATEGY=rule_based REPLAY=1 SET="DIAG_LOG_LEVEL=1 FRICTION_MODE='random' PROBE_ENABLED=True PROBE_END_PULLS=True PROBE_EDGE_PUSH=True" sbatch -t 03:00:00 -a 583 cluster/jenga_array.sbatch
