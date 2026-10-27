"""Sanity checks on config.py.  Pure Python: no Drake needed.

check_tags(path)   every setting must end with a source tag: [user] [course] [measured] [derived] [estimate]
check_values(cfg)  settings that cannot work, or that probably will not
describe(cfg)      the numbers a person wants to see (block size, margins, mass)
"""

import re
from pathlib import Path

from jenga import geometry as geo

TAGS = ("[user]", "[course]", "[measured]", "[derived]", "[estimate]")
_ASSIGN = re.compile(r"^([A-Z][A-Z0-9_]*)\s*=")


def check_tags(path) -> list:
    """Warnings for any setting line in the file that has no source tag in its comment."""
    out = []
    for n, line in enumerate(Path(path).read_text().splitlines(), 1):
        m = _ASSIGN.match(line)
        if m and not any(t in line for t in TAGS):
            out.append(f"config.py line {n}: {m.group(1)} has no source tag ({', '.join(TAGS)}).")
    return out


def check_values(cfg) -> list:
    w = []
    choices = {
        "MOVER": ("fake", "real"), "EXTRACTION_METHOD": ("end_pinch", "push"),
        "STRATEGY": ("random", "rule_based", "perception", "force_guided"),
        "COLLAPSE_DETECTOR": ("basic", "advanced"), "BASE_MODE": ("mobile", "fixed"),
        "CONTACT_MODEL": ("hydroelastic", "point"), "FRICTION_MODE": ("global", "random"),
        "CONTACT_APPROXIMATION": ("sap", "lagged", "similar", "default"),
        "FIRST_LAYER_LONG_AXIS": ("x", "y"),
    }
    for name, allowed in choices.items():
        if getattr(cfg, name) not in allowed:
            w.append(f"{name} must be one of {allowed}, got {getattr(cfg, name)!r}.")
    L, W, H = geo.block_dims(cfg)
    if cfg.BLOCK_SCALE <= 0:
        w.append("BLOCK_SCALE must be positive.")
    jaw = geo.jaw_margin_per_side_m(cfg)
    fin = geo.finger_height_margin_per_side_m(cfg)
    if jaw < 0:
        w.append(f"The block ({L * 1000:.1f} mm long) does not fit between the open pads ({geo.pad_gap_m(cfg) * 1000:.1f} mm). Lower BLOCK_SCALE.")
    elif jaw < 0.002:
        w.append(f"Jaw clearance is only {jaw * 1000:.2f} mm per side (derived): a tight fit.")
    if fin < 0:
        w.append(f"The block ({H * 1000:.1f} mm tall) is lower than the fingers ({cfg.FINGER_HEIGHT_M * 1000:.0f} mm): the pads will touch the layers above and below. Raise BLOCK_SCALE.")
    elif fin < 0.0005:
        w.append(f"Finger-height clearance is only {fin * 1000:.2f} mm per side (derived): the hand must be that accurate in height.")
    if cfg.FINGER_OPEN_COMMAND_M > 0.110 + 1e-9:
        w.append("FINGER_OPEN_COMMAND_M is above the gripper's 110 mm maximum.")
    if geo.block_mass(cfg) > 0.1:
        w.append(f"Block mass {geo.block_mass(cfg):.3f} kg is above the 0.1 kg the psets hold (estimate).")
    if len(cfg.ARM_KP) != 7 or len(cfg.ARM_KD) != 7 or len(cfg.BASE_KP) != 3 or len(cfg.BASE_KD) != 3:
        w.append("ARM_KP/ARM_KD need 7 entries and BASE_KP/BASE_KD need 3.")
    if cfg.PLANT_TIME_STEP is None or cfg.PLANT_TIME_STEP <= 0:
        w.append("PLANT_TIME_STEP must be a positive number (the plant is built by hand and needs one).")
    if cfg.N_LAYERS_START < 2:
        w.append("N_LAYERS_START must be at least 2: there is nothing legal to pull from one layer.")
    if cfg.MAX_TURNS < 1 or cfg.N_TRIALS < 1:
        w.append("MAX_TURNS and N_TRIALS must be at least 1.")
    if cfg.PLATFORM_HEIGHT_M < 0:
        w.append("PLATFORM_HEIGHT_M cannot be negative.")
    if cfg.BASE_TRAVEL_RADIUS_M < cfg.STATION_STANDOFF_M:
        w.append("BASE_TRAVEL_RADIUS_M is smaller than STATION_STANDOFF_M: the base would roll through the tower.")
    if len(cfg.STATION_ANGLES_DEG) < 1:
        w.append("STATION_ANGLES_DEG needs at least one station.")
    if cfg.EXTRACTION_METHOD == "push":
        w.append("EXTRACTION_METHOD = 'push' is not implemented (placeholder).")
    if cfg.MOVER == "fake":
        w.append("MOVER = 'fake': runs use the teleport placeholder and are INVALID for results (this is the default until the team's steps exist).")
    return w


def describe(cfg) -> str:
    L, W, H = geo.block_dims(cfg)
    return "\n".join([
        f"block         : {L * 1000:.2f} x {W * 1000:.2f} x {H * 1000:.2f} mm  (scale {cfg.BLOCK_SCALE}, real {', '.join(f'{v * 1000:.0f}' for v in cfg.BLOCK_REAL_M)} mm)",
        f"mass          : {geo.block_mass(cfg):.3f} kg  [estimate: density x volume]",
        f"jaw           : pad gap {geo.pad_gap_m(cfg) * 1000:.1f} mm; clearance {geo.jaw_margin_per_side_m(cfg) * 1000:.2f} mm per side  [derived]",
        f"finger height : fingers {cfg.FINGER_HEIGHT_M * 1000:.0f} mm in a {H * 1000:.2f} mm block; clearance {geo.finger_height_margin_per_side_m(cfg) * 1000:.2f} mm per side  [derived]",
        f"tower         : {cfg.N_LAYERS_START} layers, footprint {geo.tower_footprint_m(cfg) * 1000:.1f} mm, on a {cfg.PLATFORM_HEIGHT_M * 1000:.0f} mm platform",
        f"robot         : base mode {cfg.BASE_MODE}, {len(cfg.STATION_ANGLES_DEG)} stations at {cfg.STATION_STANDOFF_M} m",
        f"run           : method {cfg.EXTRACTION_METHOD}, strategy {cfg.STRATEGY}, mover {cfg.MOVER}, contact {cfg.CONTACT_MODEL}",
    ])
