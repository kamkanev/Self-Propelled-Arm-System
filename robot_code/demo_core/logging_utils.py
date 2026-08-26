from __future__ import print_function

import os
import sys
import time

from .config import LOG_DIR


class TeeStream(object):
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for stream in self.streams:
            stream.write(data)
            stream.flush()

    def flush(self):
        for stream in self.streams:
            stream.flush()

    def isatty(self):
        return False


class TeeLogger(object):
    def __init__(self, path):
        self.path = path
        self.file = None
        self.old_stdout = None
        self.old_stderr = None

    def __enter__(self):
        log_dir = os.path.dirname(self.path)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
        self.file = open(self.path, "a", buffering=1)
        self.old_stdout = sys.stdout
        self.old_stderr = sys.stderr
        sys.stdout = TeeStream(self.old_stdout, self.file)
        sys.stderr = TeeStream(self.old_stderr, self.file)
        print("[log] writing {}".format(self.path))
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            print("[log] closing {}".format(self.path))
        finally:
            sys.stdout = self.old_stdout
            sys.stderr = self.old_stderr
            self.file.close()


def default_log_path(prefix="demo"):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(str(LOG_DIR), "{}_{}.log".format(prefix, stamp))
