# workfiles: patches kept for later

Each `.patch` here is a change that is **not** in the code on purpose, kept so nobody has to rebuild it.  Each file
starts with a note: what it does, why it exists, and how to apply it.  Apply with `git apply workfiles/<name>.patch`
(check first with `git apply --check`), and take it out again with `git apply -R ...` before committing.

| Patch | What it is | Status |
|---|---|---|
| `held_swing_testing.patch` | Push method: slow the swing while a block is held (`HELD_SWING_MAX_SPEED_MS`) | Tried, mixed result; to re-test later (DECISIONS O-5) |
| `push_end_testing.patch` | Push method: push from the `"far"` end as well as the default `"near"` one (`PUSH_END`) | Debugging switch |
| `end_heights_testing.patch` | Development log: how far each end of the moving block rose (`inner_end_up_mm`, `outer_end_up_mm`) | Debugging columns |
