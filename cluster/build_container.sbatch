#!/bin/bash
#SBATCH -J jenga-build
#SBATCH -p mit_normal
#SBATCH -t 01:00:00
#SBATCH -c 4
#SBATCH --mem=8G
#SBATCH -o logs/%x-%j.out

module load apptainer/1.4.2
apptainer build --force --fakeroot jenga.sif cluster/jenga.def
