# Copyright (c) 2020, CNRS
# Authors: Guilhem Saurel <guilhem.saurel@laas.fr>

import os
import subprocess
import time

import hpp.corbaserver

try:
    from subprocess import DEVNULL, run
except ImportError:  # Python2 fallback
    DEVNULL = os.open(os.devnull, os.O_RDWR)

    def run(*args):
        subprocess.Popen(*args).wait()


class ServerManager:
    """A context to ensure a server is running."""

    def __init__(self, server="hppcorbaserver"):
        self.server = server
        run(["killall", self.server])

    def __enter__(self):
        """Run the server in background

        stdout and stderr outputs of the child process are redirected to devnull.
        preexec_fn is used to ignore ctrl-c signal send to the main script
        (otherwise they are forwarded to the child process)
        """
        self.process = subprocess.Popen(
            self.server, stdout=DEVNULL, stderr=DEVNULL, preexec_fn=os.setpgrp
        )
        # give it some time to start
        time.sleep(3)

    def __exit__(self, exc_type, exc_value, exc_traceback):
        tool = hpp.corbaserver.tools.Tools()
        tool.shutdown()
        # Give some time to HPP to properly shutdown.
        time.sleep(1)
        # Once HPP process is stopped, this removes the defunct process.
        self.process.communicate()
