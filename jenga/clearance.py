"""ARM CLEARANCE: TODO for the team (DECISIONS.md, open items).

The arm model has NO collision geometry (measured from the model's file name), so the simulator will not
stop the arm passing through the table, the platform or the tower.  Nothing here checks for that yet.
Options the team can weigh: an iiwa model with collision geometry (check whether one ships with Drake),
geometric proxies (capsules per link) checked on each planned path, or watching the replays.

Two hooks exist so the decision can be made later without restructuring:
  * check_motion() is called once per motion by the executor.  It does nothing yet.
  * ARM_COLLISIONS_CHECKED feeds the CSV column `arm_collisions_unchecked`.  Set it True ONLY when
    check_motion really checks, so a run is never mistaken for a checked one.
"""

ARM_COLLISIONS_CHECKED = False  # leave False until check_motion() below really checks


def check_motion(world, waypoints, label: str = "") -> None:
    """Called before each motion.  Raise jenga.errors.MotionFailed if the planned motion would hit something.

    Inputs: world (the Sim), waypoints (list of Waypoint in the robot frame), label (text for messages).
    """
    # ---- BEGIN CODE TO BE WRITTEN ----
    return None  # TODO: the team decides how (or whether) to check; until then nothing is checked
    # ---- END CODE TO BE WRITTEN ----
