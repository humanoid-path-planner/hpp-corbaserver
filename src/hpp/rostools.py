from __future__ import print_function
import sys
from xacro import filestack
import xml

try:  # python 2
    _basestr = basestring
    encoding = {"encoding": "utf-8"}
except NameError:  # python 3
    _basestr = str
    unicode = str
    encoding = {}


def process_xacro(*args):
    import xacro

    opts, input_file_name = xacro.process_args(args)
    try:
        # open and process file
        doc = xacro.process_file(retrieve_resource(input_file_name), **vars(opts))

    # error handling
    except xml.parsers.expat.ExpatError as e:
        xacro.error("XML parsing error: %s" % unicode(e), alt_text=None)
        if xacro.verbosity > 0:
            xacro.print_location(filestack, e)
            print(file=sys.stderr)  # add empty separator line before error
            print("Check that:", file=sys.stderr)
            print(" - Your XML is well-formed", file=sys.stderr)
            print(
                " - You have the xacro xmlns declaration:",
                'xmlns:xacro="http://www.ros.org/wiki/xacro"',
                file=sys.stderr,
            )
        sys.exit(2)  # indicate failure, but don't print stack trace on XML errors

    except Exception as e:
        msg = unicode(e)
        if not msg:
            msg = repr(e)
        xacro.error(msg)
        if xacro.verbosity > 0:
            xacro.print_location(filestack, e)
        if xacro.verbosity > 1:
            print(file=sys.stderr)  # add empty separator line before error
            raise  # create stack trace
        else:
            sys.exit(2)  # gracefully exit with error condition

    # write output
    return doc.toprettyxml(indent="  ", **encoding)


def retrieve_resource(path, dirs=None, env_var="ROS_PACKAGE_PATH"):
    """
    Retrieve resource of the form "package://", resolving the package in the list of
    dirs.
    If the list of dirs is None, it is initialized with
    the content of the environnement variable env_var.
    """
    if path.startswith("package://"):
        relpath = path[len("package://") :]
        import os

        if dirs is None:
            dirs = os.environ[env_var].split(":")
        for dir in dirs:
            abspath = os.path.join(dir, relpath)
            if os.path.isfile(abspath):
                return abspath
    return path
