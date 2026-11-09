# Running trials on MIT ORCD (Engaging)

Each trial runs as its own Slurm job (one core each), so trials run in parallel.
Drake is not thread-safe, so we never split one trial across cores.

## One-time setup (each teammate, on their own ORCD account)

1. Log in once to the ORCD OnDemand portal with your Kerberos ID, then ssh in:

        ssh YOURKERB@orcd-login.mit.edu

2. Clone the repo into a path with no spaces and make the log folder:

        git clone https://github.com/seetha15-mit/jenga-iiwa
        cd jenga-iiwa
        git checkout seetha/speed-up-wall-time
        mkdir -p logs

3. Find your Slurm account and set it once (ours is mit_general; check yours):

        sacctmgr show assoc user=$USER format=User,Account%30,Partition%30
        echo 'export SALLOC_ACCOUNT=YOUR_ACCOUNT SBATCH_ACCOUNT=YOUR_ACCOUNT' >> ~/.bashrc
        source ~/.bashrc

4. Build the container (10-20 min, runs by itself). Wait until jenga.sif exists:

        sbatch cluster/build_container.sbatch
        squeue --me
        ls -lh jenga.sif

5. Test with the fake mover (results are INVALID; this only checks that Drake starts):

        S=$(readlink -f ~/orcd/scratch); mkdir -p $S/jenga_test
        sbatch -p mit_normal -t 00:30:00 -c 1 --mem=2G -o logs/fake-test-%j.out --wrap \
          "module load apptainer/1.4.2; apptainer exec -B $S jenga.sif python run_one_trial.py --strategy random --trial 0 --mover fake --turns 1 --out-root $S/jenga_test"
        cat logs/fake-test-*.out
        rm -rf $S/jenga_test

   Pass = a "[trial 0] done" line and no traceback.

## Run trials

The only command you run is: sbatch cluster/jenga_array.sbatch

Settings go in --export: STRATEGY= , TURNS= (max turns per trial), REPLAY= (1 on, 0 off),
OUT_ROOT= (output folder).
Defaults: strategy random, turns from config.py, replay ON, output ~/orcd/scratch/jenga_scratch.

### Example: 3 trials, 3 turns each, replay on

        sbatch -a 0-2 --export=ALL,TURNS=3,REPLAY=1 cluster/jenga_array.sbatch

-a 0-2 runs trials 0, 1 and 2 in parallel. Watch them with:

        squeue --me
        tail -f $(ls -t logs/jenga-*.out | head -1)
        sacct -j JOBID -o JobID,State,Elapsed,MaxRSS,NodeList

(Each job shows 3 rows in sacct; read the first row of each group.)

## Where the output goes

- Logs: logs/jenga-JOBID_TRIAL.out
- Results: ~/orcd/scratch/jenga_scratch/STRATEGY_trialNN/ containing turns.csv,
  trials.csv (only if the trial finished) and trial_NN_replay.html (if replay was on)
- Find replays:

        find $(readlink -f ~/orcd/scratch) -name "*_replay.html"

## Copy results to your laptop and watch a replay

~/orcd/scratch is a symlink, so rsync needs the full real path. Get it on the cluster:

        readlink -f ~/orcd/scratch

Copy the output (something like /orcd/scratch/.../yourname). To find the folder of a
specific replay:

        dirname $(find $(readlink -f ~/orcd/scratch) -name "*_replay.html" | head -1)

Then, on your laptop (not logged into the cluster), paste the path after the colon.
Use the folder path, not the .html file name:

        rsync -av YOURKERB@orcd-login.mit.edu:PASTE_FULL_PATH/jenga_scratch/ ./runs_scratch/

Keep the trailing / after jenga_scratch. Then open trial_NN_replay.html in a browser.

## Scratch runs vs. the final run

Everything above writes to the throwaway jenga_scratch folder. Before the final run, delete it:

        rm -rf $(readlink -f ~/orcd/scratch)/jenga_scratch

Final run: time one 6-layer trial first (replay off) and read its MaxRSS with sacct, then set
--mem to about double that. Set N_LAYERS_START = 6 in config.py, then:

        sbatch --mem=MEM -t HH:MM:SS --export=ALL,REPLAY=0,TURNS=50,OUT_ROOT=$(readlink -f ~/orcd/scratch)/jenga_final cluster/jenga_array.sbatch

Then run: python make_results.py FOLDER
It averages everything in that folder, so only point it at final runs, and check that the
"trials" column matches the number you launched.

## Troubleshooting

- "Invalid account or account/partition": set your Slurm account (setup step 3).
- "File exists: .../orcd/scratch" inside the container: the scratch path is a symlink; the
  scripts already use readlink -f.
- "libglib ... cannot open shared object file": a system library is missing in
  cluster/jenga.def; add it to the apt-get install line and rebuild.
- A trial folder with turns.csv but no trials.csv did not finish (timeout, crash or
  out-of-memory). Check sacct, fix, and resubmit the same command; finished trials are skipped.
