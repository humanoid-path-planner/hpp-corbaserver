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
    class _ToolClient(Client):
        def __init__(self, url = None):
            self._initOrb (url)

    _tc = _ToolClient(url=url)
    client = _tc._tools

    def deleteServantFromObject (*args):
        """
        delete a servant from an object (and not an IOR as with deleteServant)
        """
        return all( [ client.deleteServant(_tc.orb.object_to_string(o)) for o in args ] )

    client.deleteServantFromObject = deleteServantFromObject

    return client

if sys.version_info.major > 2:
    Tools.__doc__ = _hpp.Tools__doc__
