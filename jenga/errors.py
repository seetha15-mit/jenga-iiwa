"""Exceptions shared across modules (plain Python)."""


class MotionFailed(RuntimeError):
    """A motion could not be carried out (for example no IK solution for a waypoint).
    The harness logs the turn as a failed turn and lets the strategy pick again."""
