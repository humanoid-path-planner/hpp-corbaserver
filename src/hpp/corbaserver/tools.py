import sys
from omniORB import CORBA
import CosNaming, hpp_idl.hpp as _hpp

from .client import CorbaError, _getIIOPurl

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
    import sys
    orb = CORBA.ORB_init (sys.argv, CORBA.ORB_ID)
    if url is None:
        ns = orb.string_to_object (_getIIOPurl())
    else:
        ns = orb.string_to_object (url)

    rootContext = ns._narrow(CosNaming.NamingContext)
    if rootContext is None:
      raise CorbaError ('failed to narrow the root context')

    serviceName = "hpp.tools"
    name = [CosNaming.NameComponent ("hpp", "tools"),]

    try:
      obj = rootContext.resolve (name)
    except CosNaming.NamingContext.NotFound as ex:
      raise CorbaError (
        'failed to find the service ``{0}\'\''.format (serviceName))

    try:
      client = obj._narrow (_hpp.Tools)
    except KeyError:
      raise CorbaError ('invalid service name ``{0}\'\''.format (serviceName))

    if client is None:
      # This happens when stubs from client and server are not synchronized.
      raise CorbaError (
        'failed to narrow client for service named ``{0}\'\''.format
        (serviceName))

    def deleteServantFromObject (*args):
        """
        delete a servant from an object (and not an IOR as with deleteServant)
        """
        return all( [ client.deleteServant(orb.object_to_string(o)) for o in args ] )

    client.deleteServantFromObject = deleteServantFromObject

    return client

if sys.version_info.major > 2:
    Tools.__doc__ = _hpp.Tools__doc__
