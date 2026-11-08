"""Supervise the official Isaac launcher and its children as one owned process group."""

import os
import signal
import subprocess
import sys
import time


def main():
    child = None
    stopping = None

    def stop(*_):
        nonlocal stopping
        if stopping is None:
            stopping = time.monotonic()
            if child is not None:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    child = subprocess.Popen(sys.argv[1:], start_new_session=True)
    if stopping is not None:
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    while child.poll() is None:
        if stopping is not None and time.monotonic() - stopping > 12:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(0.05)
    # The shell may exit on TERM before Kit finishes. Wait for the retained
    # group, then apply the same bounded shutdown to its remaining children.
    if stopping is not None:
        while True:
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                break
            if time.monotonic() - stopping > 12:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                break
            time.sleep(0.05)
    return 0 if stopping is not None else child.returncode


if __name__ == "__main__":
    sys.exit(main())
