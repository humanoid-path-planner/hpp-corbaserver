"""
Provide a client for CORBA services which initialize CORBA automatically and
create client to wanted HPP services.
"""

import typing

import CosNaming
import hpp_idl.hpp.corbaserver
from omniORB import CORBA


class CorbaError(Exception):
    """
    Raised when a CORBA error occurs.
    """

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class Client:
    """
    Connect and create clients for all HPP services.
    """

    defaultPort = 13331
    defaultClients: typing.ClassVar = {
        "problem": hpp_idl.hpp.corbaserver.Problem,
        "obstacle": hpp_idl.hpp.corbaserver.Obstacle,
        "robot": hpp_idl.hpp.corbaserver.Robot,
    }

    def _makeClient(self, serviceId, serviceName, class_, mainContext):
        """
        Create a client to a new CORBA service and add it to this class.
        """
        obj = self._tools.getServer(mainContext, serviceId, serviceName)
        client = obj._narrow(class_)

        if client is None:
            # This happens when stubs from client and server are not synchronized.
            raise CorbaError(
                'Service "{}" is not of type {}'.format(
                    ".".join([mainContext, serviceId, serviceName]), class_
                )
            )
        self.__dict__[serviceName] = client

    def initWithNameService(self, urlNameService):
        obj = self.orb.string_to_object(urlNameService)
        self.rootContext = obj._narrow(CosNaming.NamingContext)
        if self.rootContext is None:
            raise CorbaError("Failed to narrow the root context")

        name = [
            CosNaming.NameComponent("hpp", "tools"),
        ]

        try:
            obj = self.rootContext.resolve(name)
        except CosNaming.NamingContext.NotFound:
            raise CorbaError('Failed to find the service "tools"')

        try:
            client = obj._narrow(hpp_idl.hpp.Tools)
        except KeyError:
            raise CorbaError('Invalid service name "hpp.tools"')

        if client is None:
            # This happens when stubs from client and server are not synchronized.
            raise CorbaError('Failed to narrow client for service named "hpp.tools"')

        self._tools = client

    def initWithDirectLink(self, url):
        obj = self.orb.string_to_object(url)
        try:
            client = obj._narrow(hpp_idl.hpp.Tools)
        except CORBA.Exception as e:
            self._tools = None
            raise CorbaError(
                "Failed to narrow client at "
                + url
                + " into hpp_idl.hpp.Tools: "
                + str(e)
            )

        if client is None:
            # This happens when stubs from client and server are not synchronized.
            raise CorbaError(
                "Failed to narrow client at " + url + " into " + "hpp_idl.hpp.Tools"
            )

        self._tools = client

    def __init__(
        self,
        clients=defaultClients,
        url=None,
        context="corbaserver",
        host=None,
        port=None,
    ):
        """
        Initialize CORBA and create default clients.
        :param url: URL in the IOR, corbaloc, corbalocs, and corbanames formats.
                    For a remote corba server, use
                    url = "corbaloc:iiop:<host>:<port>/NameService"
        :param host: if not None, url is set to :
                    "corbaloc:iiop:" + str(host) + "/NameService"
        """
        self._initOrb(url, host, port)
        self._makeClients("", clients, context)

    def _initOrb(self, url, host=None, port=None):
        import sys

        self.orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

        if url is not None:
            if url.endswith("/NameService"):
                self.initWithNameService(url)
            elif url.endswith("/hpp-corbaserver"):
                self.initWithNameService(url)
            else:
                try:
                    self.initWithDirectLink(url + "/hpp-corbaserver")
                except CorbaError:
                    pass
                if self._tools is None:
                    self.initWithNameService(url + "/NameService")
        else:
            urlNameService = _getIIOPurl(
                service="NameService", host=host, port=port, default_port=2809
            )
            urlHppTools = _getIIOPurl(
                service="hpp-corbaserver",
                host=host,
                port=port,
                default_port=self.defaultPort,
            )
            try:
                self.initWithDirectLink(urlHppTools)
            except CorbaError as e:
                print(e)
                pass
            if self._tools is None:
                self.initWithNameService(urlNameService)

    def _makeClients(self, serviceId, clients, context):
        for serviceName, class_ in clients.items():
            self._makeClient(serviceId, serviceName, class_, context)


def _getIIOPurl(service="NameService", host=None, port=None, default_port=None):
    """
    Returns "corbaloc:iiop:<host>:<port>/NameService"
    where host and port are, in this order of priority:
    - HPP_HOST, HPP_PORT environment variables
    - /hpp/host, /hpp/port ROS parameters
    - use default values ("localhost", 2809)
    """
    _host = "localhost"
    _port = None
    import os

    try:
        import socket  # noqa: F401

        import rospy

        # Check is ROS master is reachable.
        if rospy.client.get_master().target is not None:
            _host = rospy.get_param("/hpp/host", _host)
            _port = rospy.get_param("/hpp/port", _port)
    except (ImportError, OSError):
        pass
    _host = os.getenv("HPP_HOST", _host)
    _port = os.getenv("HPP_PORT", _port)
    if host:
        _host = host
    if port:
        _port = port
    if _port is None:
        _port = default_port
    if _host is None and _port is None:
        url = "corbaloc:iiop:"
    else:
        url = f"corbaloc:iiop:{_host}:{_port}"
    return url + "/" + service
