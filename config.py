"""ALL TUNABLE VALUES LIVE HERE.  Edit this file, nothing else, to change a setting.

Every line ends with a tag saying where the number came from:
    [user]       you (the team) decided it
    [course]     the course page / proposal / psets say so
    [measured]   read from a model file or a run on our machines
    [derived]    worked out from measured values; not tested in simulation
    [estimate]   a guess.  Replace it with a measured value when you can.
`python run_checks.py` warns about any line without a tag.

Units: meters, kilograms, seconds, radians unless the name says otherwise.
Frames: W = world (tower centre at the origin, table top at z = 0).
        R = robot base frame (x forward, z up).  G = gripper frame
        (x = finger axis, y = reach axis toward the fingertips, z = the third axis).
"""

# ------------------------------------------------------------------ what to run
N_LAYERS_START = 3  # [course] start with 3 layers; the TA said make 3 work before 12
N_TRIALS = 10  # [course] at least 10 trials per approach
BASE_SEED = 1000  # [estimate] trial i uses seed BASE_SEED + i (first block + hidden friction)
MAX_TURNS = 10  # [estimate] a trial ends after this many turns even without a collapse
MAX_CONSECUTIVE_FAILURES = 2  # [estimate] a trial ends after this many failed turns in a row
START_SETTLE_S = 1.0  # [estimate] the tower stands this long before the first turn, so a replay opens on the intact tower
MOVER = "fake"  # [user] "fake" = teleport placeholder (results INVALID), "real" = the team's steps
EXTRACTION_METHOD = "end_pinch"  # [user] "end_pinch" (default) or "push" (push, pull, pedestal regrab, place; D-21)
STRATEGY = "random"  # [course] "random", "rule_based", "perception", "force_guided"
COLLAPSE_DETECTOR = "basic"  # [user] "basic" or "advanced" (the team's)

# ------------------------------------------------------------------ blocks
BLOCK_REAL_M = (0.075, 0.025, 0.015)  # [course] a real Jenga block: length, width, height
BLOCK_SCALE = 1.45  # [user] ONE scale for all three dimensions (decided from the gripper numbers)
BLOCK_GAP_M = 0.0005  # [estimate] gap between neighbouring blocks (prototype value, not verified)
BLOCK_DENSITY = 700.0  # [estimate] kg/m^3; gives about 0.06 kg per block at this scale
FRICTION_MODE = "global"  # [user] "global" (all equal) or "random" (varied, hidden from strategies)
FRICTION_GLOBAL = 0.5  # [estimate] used when FRICTION_MODE = "global"
FRICTION_RANGE = (0.3, 0.7)  # [estimate] used when FRICTION_MODE = "random"
HYDROELASTIC_MODULUS = 1.0e6  # [course] what the psets use for their boxes

# ------------------------------------------------------------------ gripper (WSG-50 with tips)
FINGER_OPEN_COMMAND_M = 0.110  # [measured] largest opening; the pad gap is this + 5 mm
PAD_GAP_EXTRA_M = 0.005  # [measured] pad gap = command + 5 mm (pads never get closer than 5 mm)
FINGER_HEIGHT_M = 0.020  # [measured] fingers and pads are 20 mm tall
FINGER_CLOSED_WIDTH_M = 0.035  # [measured] two closed fingers together
PAD_CENTRE_FROM_G_M = 0.108  # [measured] distance from G to the middle of the pads along G y
FINGERTIP_REACH_M = 0.118  # [measured] distance from G to the fingertips along G y
HAND_HOUSING_M = (0.146, 0.0725, 0.050)  # [measured] hand body size along G x, y, z
HAND_HOUSING_FRONT_Y_M = 0.0363  # [measured] front face of the hand body, along G y from G
FINGER_SQUEEZE_M = 0.02  # [estimate] how much narrower than the block to command (pset practice)
WRIST_REST_FORCE_N = 10.7  # [measured] wrist force magnitude at rest, hand pointing down
HOUSING_CLEARANCE_M = 0.005  # [estimate] extra room wanted between hand body and tower when placing

# ------------------------------------------------------------------ platform, table, tower
TOWER_CENTER_XY = (0.0, 0.0)  # [user] the tower stays here; the robot moves around it
PLATFORM_HEIGHT_M = 0.10  # [estimate] top of the platform above the table; the team sweeps this
PLATFORM_MARGIN_M = 0.0  # [estimate] platform sticks out this far past the tower on each side
FIRST_LAYER_LONG_AXIS = "y"  # [user] layer 0 blocks lie along world y; layers alternate
TABLE_SIZE_M = (2.5, 2.5, 0.05)  # [estimate] table top is at z = 0
TABLE_FRICTION = 0.5  # [estimate]
# regrab pedestals (push method): four posts on the diagonals between stations, where a pulled block is set down
# and picked up again by its ends with a level hand (DECISIONS.md D-22)
PEDESTAL_DISTANCE_M = 0.27  # [derived] tower centre to post centre along a diagonal; IK-only: 29 deg worst joint margin
PEDESTAL_HEIGHT_M = 0.07  # [derived] top above the table: 16 mm under the lowest hand body near the tower (layer 0)
PEDESTAL_TOP_M = 0.03  # [derived] square top, narrower than a block (36 mm) so the fingers pass beside it

# ------------------------------------------------------------------ robot base and stations
BASE_MODE = "mobile"  # [user] "mobile" (rolling base) or "fixed" (welded at station 0; isolates base problems)
STATION_ANGLES_DEG = (0.0, 90.0, 180.0, 270.0)  # [estimate] directions the robot faces the tower from
STATION_STANDOFF_M = 0.95  # [user] base axis to tower centre; the reachability check passed 0.90 to 1.00 m with 16 to 31 deg margin
BASE_TRAVEL_RADIUS_M = 1.2  # [estimate] the base circles the tower at this distance between stations
BASE_SPEED_MS = 0.4  # [estimate] rolling speed
BASE_MOUNT_HEIGHT_M = 0.0  # [estimate] height of the arm's mounting flange above the table
BASE_KP = (1.0e5, 1.0e5, 2.0e4)  # [estimate] base stiffness for x, y, yaw (N/m, N/m, Nm/rad)
BASE_KD = (2.0e3, 2.0e3, 5.0e2)  # [estimate] base damping for x, y, yaw
ARM_CARRY_Q = (0.0, 0.4, 0.0, -1.6, 0.0, 1.2, 0.0)  # [measured] the psets' home pose; used while rolling

# ------------------------------------------------------------------ physics
CONTACT_MODEL = "hydroelastic"  # [estimate] "hydroelastic" or "point"; TODO compare (DECISIONS.md)
CONTACT_APPROXIMATION = "lagged"  # [measured] "sap", "lagged", "similar" or "default" (Drake's own); with "sap" a pull dragged the layer above out with the block (Drake documents SAP 'gliding' in sliding contact), with "lagged" it came out cleanly (DECISIONS.md D-10)
PLANT_TIME_STEP = 0.001  # [estimate] seconds per physics step; try 0.002 etc. when testing speed
ARM_KP = (1.0e4, 1.0e4, 1.0e4, 1.0e4, 5.0e3, 3.0e3, 2.0e3)  # [estimate] arm joint stiffness (Nm/rad); the arm is gravity-compensated (see DECISIONS.md)
ARM_KD = (500.0, 500.0, 300.0, 300.0, 100.0, 60.0, 40.0)  # [estimate] arm joint damping (Nm s/rad)
FINGER_KP = 2000.0  # [estimate] finger stiffness (N/m): grip force = this x how far past the block the fingers are commanded
FINGER_KD = 50.0  # [estimate] finger damping (N s/m)

# ------------------------------------------------------------------ motions
WAYPOINT_SPEED_MS = 0.1  # [estimate] hand speed between waypoints
JOINT_SPEED_RAD_S = 0.5  # [estimate] speed of joint-space moves
HOLD_AFTER_MOTION_S = 0.3  # [estimate] stand still this long after each motion
SAMPLE_DT_S = 0.02  # [estimate] how often to log wrist force during a motion
TRACKING_WARN_M = 0.005  # [estimate] warn if the hand ends this far from its last waypoint
JOINT_LIMIT_WARN_RAD = 0.05  # [estimate] warn if an arm joint ends this close to its limit

# ------------------------------------------------------------------ turn outcomes and collapse
COLLAPSE_DISPLACEMENT_M = 0.02  # [estimate] any other block moving more than this ends the trial
PLACE_TOLERANCE_M = 0.02  # [estimate] a block this close to its target slot counts as replaced
PULL_MOVED_M = 0.02  # [estimate] a block that moved less than this was not pulled out

# ------------------------------------------------------------------ replay and logs
RECORD_REPLAY = True  # [user] record each trial for a normal-speed replay afterwards
RUNS_DIR = "runs"  # [user] real results go here
FAKE_RUNS_DIR = "runs_fake"  # [user] fake-mover runs go here, never into results
ASSET_DIR = "generated_assets"  # [estimate] SDF files written at run time
PRINT_SIM_SPEED = True  # [user] print simulated seconds per wall second

# ------------------------------------------------------------------ sensing (notes only; the team decides)
ENABLE_WRIST_FORCE_LOG = False  # [user] log the raw wrist force during motions
ENABLE_CAMERAS = False  # [user] cameras are not set up; see jenga/sensing.py notes

# ------------------------------------------------------------------ run_checks.py
STATIC_CHECK_S = 3.0  # [measured] the prototype used 3 s of rest for the stable-tower check
STATIC_MAX_MOVE_M = 0.002  # [estimate] a block moving more than this during the check fails it
REACH_CHECK_LAYERS = 6  # [user] all of these layers (plus the placement level) must be reachable
MIN_JOINT_MARGIN_DEG = 10.0  # [estimate] reachability check wants at least this from any joint limit
