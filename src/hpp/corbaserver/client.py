"""
Provide a client for CORBA services which initialize CORBA automatically and
create client to wanted HPP services.
"""
from omniORB import CORBA
import CosNaming

import hpp.corbaserver

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

  defaultClients = ['problem', 'obstacle', 'robot']

  def makeClient(self, serviceName):
    """
    Create a client to a new CORBA service and add it to this class.
    """
    serviceName = serviceName.lower ()
    name = [CosNaming.NameComponent ("hpp", "corbaserver"),
            CosNaming.NameComponent ("basic", serviceName)]

    try:
      obj = self.rootContext.resolve (name)
    except CosNaming.NamingContext.NotFound, ex:
      raise CorbaError (
        'failed to find the service ``{0}\'\''.format (serviceName))

    try:
      client = obj._narrow (hpp.corbaserver.__dict__[serviceName.capitalize ()])
    except KeyError:
      raise CorbaError ('invalid service name ``{0}\'\''.format (serviceName))

    if client is None:
      # This happens when stubs from client and server are not synchronized.
      raise CorbaError (
        'failed to narrow client for service named ``{0}\'\''.format
        (serviceName))
    self.__dict__[serviceName] = client


  def __init__(self, clients = defaultClients, url = "corbaloc:rir:/NameService"):
    """
    Initialize CORBA and create default clients.
    :param url: URL in the IOR, corbaloc, corbalocs, and corbanames formats.
                For a remote corba server, use
                url = "corbaloc:iiop:<host>:<port>/NameService"
    """
    import sys
    self.orb = CORBA.ORB_init (sys.argv, CORBA.ORB_ID)
    obj = self.orb.string_to_object (url)
    self.rootContext = obj._narrow(CosNaming.NamingContext)
    if self.rootContext is None:
      raise CorbaError ('failed to narrow the root context')

    for client in clients:
      self.makeClient (client)
