"""
Provides an inter-process file lock for synchronizing access to shared memory.

This module implements a named, advisory lock using fcntl.flock() to coordinate
access to shared memory buffers across multiple Python processes.

All processes accessing the same shared memory buffers MUST acquire the same
named lock before reading or modifying shared data. Failure to do so may
result in inconsistent data, partially updated buffers, or mismatched frames.

The lock supports explicit acquisition and release, as well as Python's
context manager protocol for automatic release.

Note:
    The standard lock name for DepthAI camera frame access is 'rover_camera'.

    Locks are advisory and only coordinate processes that explicitly use them.
    This implementation requires a Unix-like operating system supporting
    fcntl.flock().
"""

import fcntl


class SharedLock:
    """
    Implements a named, inter-process lock using filesystem locking.

    Processes using the same lock name synchronize through a common lock
    file located in /tmp. Exclusive access is granted to one process at
    a time, blocking other cooperating processes until the lock is released.

    Attributes:
        lock_file:
            File object used to maintain the underlying filesystem lock.
    """

    def __init__(self, name: str):
        """
        Initializes a named lock.

        Args:
            name:
                Name identifying the lock shared between processes.
        """
        self.lock_file = open("/tmp/" + name + ".lock", "a+b")

    def acquire(self):
        """
        Acquires exclusive access to the named lock.

        Blocks until the lock becomes available.
        """
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)

    def release(self):
        """
        Releases the currently held lock.

        Allows another waiting process to acquire exclusive access.
        """
        fcntl.flock(self.lock_file, fcntl.LOCK_UN)

    def __enter__(self):
        """
        Acquires the lock upon entering a context manager.

        Returns:
            SharedLock:
                The acquired lock instance.
        """
        self.acquire()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Releases the lock upon exiting a context manager.

        The lock is released regardless of whether an exception occurred.

        Args:
            exc_type:
                Type of exception raised, if any.
            exc_value:
                Exception instance, if any.
            traceback:
                Traceback associated with the exception, if any.
        """
        self.release()

    def close(self):
        """
        Closes the underlying lock file.

        Any lock held through this file descriptor is released.
        The instance should not be used after closing.
        """
        self.lock_file.close()
