JENGA FINAL RUN: REPLAYS FOR THE VIDEO
======================================

44 replays of games from the final run (code commit 7345e58, trials 500-599).
Every replay was rerun and checked: it ends the same way as the original game
(same number of blocks, same ending).


WHAT IS IN THIS FOLDER
----------------------

README.txt        This file.

videos/           One file per game, named <condition>_<strategy>_trial<number>.html
                  e.g. B_rule_based_trial517.html is condition B, trial 517.
                  Open it in a web browser (Chrome or Firefox). It is a 3D
                  replay: drag to rotate, scroll to zoom, and use the
                  Animations controls in the panel on the right to play it.

replays.csv       One row per video. Open it in Excel / Google Sheets.
                  Columns:
                    video_file             the file in videos/
                    condition              which of the 8 conditions (A-H, below)
                    why_chosen             why this game was picked (see "How the
                                           games were picked")
                    trial                  the trial number (500-599)
                    blocks_replaced        blocks the robot moved to the top before
                                           the tower fell or the game ended
                    strategy               random or rule_based
                    settings               the config settings the game ran with
                    recreate_on_orcd       the command that makes this replay again
                                           on ORCD (see "Making a replay again")
                    seed                   the random seed (1000 + trial)
                    code_commit            the exact code version
                    container_lock_sha256  which Python packages the container had

recreate_all.sh   All 44 recreate commands in one script, one line per video,
                  with a comment saying which video each line makes.


THE 8 CONDITIONS
----------------

Each condition played the same 100 towers (trials 500-599). The same trial
number means the same starting tower in every condition, so two conditions can
be compared game by game.

  A  random                     Picks a random block each turn.
  B  rule-based                 Picks blocks by our rules (safest first).
  C  rule-based + perception    Same as B, but sees the tower through simulated
                                cameras instead of being told where blocks are.
  D  rule-based, varied friction
                                Same as B, but every block has its own friction
                                (0.3-0.7), like a real worn set. No probe.
  E  rule-based + probe         Same as D, plus the force probe: before pushing
                                a middle block, the robot pushes it 2 mm and
                                feels the resistance. Too stiff (over 5.1 N):
                                it leaves that block and picks another.
  F  random, varied friction    Same as A, with varied friction. No probe.
  G  random + probe             Same as F, plus the middle-block probe from E.
  H  rule-based + probe on all blocks
                                Same as E, but also probes edge blocks (a small
                                fingertip push, rejected over 3.9 N).


THE COMPARISONS (what each one asks)
------------------------------------

  M1  B vs A   Is rule-based better than random?
               Yes: 6.14 vs 2.65 blocks per game (+3.49).

  M2  C vs B   Does seeing through cameras cost the rule-based robot anything?
               No: 6.04 vs 6.14 (-0.10), within the half-block margin we set
               in advance, so the two count as equivalent.

  M3  E vs D   With varied friction, does probing middle blocks help the
               rule-based robot?
               No difference found: 5.89 vs 5.88 (+0.01).

  M4  G vs F   Same question for the random robot.
               No difference found: 2.76 vs 2.61 (+0.15; close, but not
               significant once corrected for testing several comparisons).

  S1  H vs D   (Secondary, extra question.) Does probing every block, edges
               too, help the rule-based robot?
               No difference found: 6.03 vs 5.88 (+0.15), and the games take
               about 50% longer.

D and F are the "no probe" baselines for the probe comparisons: they have the
same varied friction as E, G and H, so the only change is the probe.


HOW THE GAMES WERE PICKED (rules fixed before the results)
----------------------------------------------------------

For each condition, three games:
  best      the game with the most blocks
  worst     the game with the fewest blocks
  typical   the game closest to the condition's median (what it usually does)

For each comparison, two pairs (the same trial in both conditions, so you can
show them side by side):
  gained the most   the trial where the second condition did best compared
                    with the first (e.g. M1, trial 517: random 1, rule-based 8)
  lost the most     the trial where it did worst (e.g. M1, trial 547: random 6,
                    rule-based 4)

Ties went to the lowest trial number. Some games were picked for more than one
reason; why_chosen lists them all. Best, worst and pairs are picked to
illustrate and are not typical; say so in the video ("longest game",
"largest difference").


MAKING A REPLAY AGAIN ON ORCD
-----------------------------

Games repeat exactly with the same code and the same container on ORCD.  A
different container (for example one built again from cluster/jenga.def) may
have slightly different system libraries, and then a game may not come out
exactly the same: a marginal grasp can go the other way.  Check the result
against blocks_replaced (step 4).

1. Get the code at the exact version:
       git clone <our repo> jenga-iiwa-replays
       cd jenga-iiwa-replays
       git checkout 7345e58
2. Put jenga.sif in that folder.  The final run's container was
       sha256sum jenga.sif
       1431ba766e8927cd7a7c4a4f7bbff950822e2c7bd71445c2038616c414141b8b
   If yours prints something else, it is a different container (see above).
3. Run the command from the recreate_on_orcd column (or bash recreate_all.sh
   for all 44) from that folder. Run mkdir -p logs first. Each command starts
   one cluster job (usually 5-20 minutes).
4. The replay appears at
       ~/orcd/scratch/jenga_replays/<condition>/<strategy>_trial<N>/trial_<N>_replay.html
   and that folder's trials.csv should show the same blocks_replaced as
   replays.csv.
