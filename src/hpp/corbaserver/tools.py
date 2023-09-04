import sys
import CORBA

import hpp_idl.hpp as _hpp
from .client import Client


def loadServerPlugin(context, plugin, url=None, port=None):
    if port is None:
        import os

        port = os.getenv("HPP_PORT", Client.defaultPort)
    client = Tools(url, port=port)
    return client.loadServerPlugin(context, plugin)


if sys.version_info.major > 2:
    loadServerPlugin.__doc__ = _hpp.Tools.loadServerPlugin__doc__


def createContext(context, url=None, port=Client.defaultPort):
    client = Tools(url, port=port)
    return client.createContext(context)


if sys.version_info.major > 2:
    createContext.__doc__ = _hpp.Tools.createContext__doc__


def Tools(url=None, port=Client.defaultPort):
    from .client import Client

    _tc = Client(url=url, clients={}, port=port)
    client = _tc._tools

    def deleteServantFromObject(*args):
        """
        delete a servant from an object (and not an IOR as with deleteServant)
        """
        return all([client.deleteServant(_tc.orb.object_to_string(o)) for o in args])

    client.deleteServantFromObject = deleteServantFromObject

    return client


class _Deleter:
    def __init__(self, o, client=None):
        orb = CORBA.ORB_init()
        self.client = client
        self.ostr = orb.object_to_string(o)
        self.type = type(o)

    def __del__(self):
        try:
            if not self.client:
                from .client import Client
                self.client = Client(clients={})._tools
            self.client.deleteServant(self.ostr)
        except CORBA.SystemException:
            pass


def wrap_delete(o, client=None):
    """
    Automatically delete the servant on the server when the Python object is deleted.
    param o the CORBA object
    param client either an instance of Client or a client to the Tools interface.
    """
    from .client import Client

    if isinstance(client, Client):
        o.__wrap_delete__ = _Deleter(o, client._tools)
    elif client is not None:
        o.__wrap_delete__ = _Deleter(o, client)
    else:
        o.__wrap_delete__ = _Deleter(o)
    return o


def equals(a, b, client=None):
    """
    Compare whether the two objects a and b are the same on the server.
    param a, b object to compare.
    param client either an instance of Client or a client to the Tools interface.
    """
    orb = CORBA.ORB_init()
    return a == b or orb.object_to_string(a) == orb.object_to_string(b)


if sys.version_info.major > 2:
    Tools.__doc__ = _hpp.Tools__doc__
