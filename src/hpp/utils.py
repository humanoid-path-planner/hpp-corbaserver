# Copyright (c) 2020, CNRS
# Authors: Guilhem Saurel <guilhem.saurel@laas.fr>

import os
import subprocess
import time


class ServerManager:
    """A context to ensure a server is running."""
    def __init__(self, server='hppcorbaserver'):
        self.server = server
        subprocess.run(['killall', self.server])

    def __enter__(self):
        """Run the server in background

        stdout and stderr outputs of the child process are redirected to devnull (hidden).
        preexec_fn is used to ignore ctrl-c signal send to the main script
        (otherwise they are forwarded to the child process)
        """
        self.process = subprocess.Popen(self.server,
                                        stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL,
                                        preexec_fn=os.setpgrp)
        # give it some time to start
        time.sleep(3)

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.process.kill()
