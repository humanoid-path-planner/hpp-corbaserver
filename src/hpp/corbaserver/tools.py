import sys
import hpp_idl.hpp as _hpp

def loadServerPlugin (context, plugin, url = None):
    client = Tools (url)
    return client.loadServerPlugin (context, plugin)
if sys.version_info.major > 2:
    loadServerPlugin.__doc__ = _hpp.Tools.loadServerPlugin__doc__

def createContext (context, url = None):
    client = Tools (url)
    return client.createContext (context)
if sys.version_info.major > 2:
    createContext.__doc__ = _hpp.Tools.createContext__doc__

def Tools(url = None):
    from .client import Client
    _tc = Client(url=url, clients={})
    client = _tc._tools

    def deleteServantFromObject (*args):
        """
        delete a servant from an object (and not an IOR as with deleteServant)
        """
        return all( [ client.deleteServant(_tc.orb.object_to_string(o)) for o in args ] )

    client.deleteServantFromObject = deleteServantFromObject

    return client

class _Deleter:
    def __init__ (self, o, client=None):
        import CORBA
        orb = CORBA.ORB_init()
        self.client = client
        self.ostr = orb.object_to_string(o)
    def __del__ (self):
        if not self.client:
            from .client import Client
            self.client = Client(clients={})._tools
        self.client.deleteServant(self.ostr)

def wrap_delete(o, client=None):
    """
    Automatically delete the servant on the server when the Python object is deleted.
    \param o the CORBA object
    \param client either an instance of Client or a client to the Tools interface.
    """
    from .client import Client
    if isinstance(client, Client):
        o.__wrap_delete__ = _Deleter(o,client._tools)
    elif client is not None:
        o.__wrap_delete__ = _Deleter(o,client)
    else:
        o.__wrap_delete__ = _Deleter(o)
    return o

if sys.version_info.major > 2:
    Tools.__doc__ = _hpp.Tools__doc__
