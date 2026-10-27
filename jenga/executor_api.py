"""Lazy access to the Drake executor, so modules that only need the pure parts import without Drake."""


def get():
    from jenga import executor

    return executor
