"""
Provide a client for CORBA services which initialize CORBA automatically and
create client to wanted HPP services.
"""
from omniORB import CORBA
import CosNaming

import hpp_idl.hpp.corbaserver

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

  defaultClients = {
          'problem' : hpp_idl.hpp.corbaserver.Problem,
          'obstacle': hpp_idl.hpp.corbaserver.Obstacle,
          'robot'   : hpp_idl.hpp.corbaserver.Robot,
          }

  def _makeClient(self, serviceId, serviceName, class_, mainContext):
    """
    Create a client to a new CORBA service and add it to this class.
    """
    serviceName = serviceName.lower ()
    name = [CosNaming.NameComponent ("hpp", mainContext),
            CosNaming.NameComponent (serviceId, serviceName)]

    fullServiceName = "/".join (["{0}.{1}".format(nc.id,nc.kind) for nc in name])

    try:
      obj = self.rootContext.resolve (name)
    except CosNaming.NamingContext.NotFound as ex:
      raise CorbaError (
        'failed to find the service ``{0}\'\''.format (fullServiceName))

    try:
      client = obj._narrow (class_)
    except KeyError:
      raise CorbaError ('invalid service name ``{0}\'\''.format (fullServiceName))

    if client is None:
      # This happens when stubs from client and server are not synchronized.
      raise CorbaError (
        'failed to narrow client for service named ``{0}\'\''.format
        (fullServiceName))
    self.__dict__[serviceName] = client

  def __init__(self, clients = defaultClients, url = None, context = "corbaserver"):
    """
    Initialize CORBA and create default clients.
    :param url: URL in the IOR, corbaloc, corbalocs, and corbanames formats.
                For a remote corba server, use
                url = "corbaloc:iiop:<host>:<port>/NameService"
    """
    self._initOrb (url)
    self._makeClients ("basic", clients, context)

  def _initOrb (self, url):
    import sys
    self.orb = CORBA.ORB_init (sys.argv, CORBA.ORB_ID)
    if url is None:
        obj = self.orb.string_to_object (_getIIOPurl())
    else:
        obj = self.orb.string_to_object (url)
    self.rootContext = obj._narrow(CosNaming.NamingContext)
    if self.rootContext is None:
      raise CorbaError ('failed to narrow the root context')

  def _makeClients (self, serviceId, clients, context):
    for serviceName, class_ in clients.items():
      self._makeClient (serviceId, serviceName, class_, context)

def _getIIOPurl ():
  """
  Returns "corbaloc:iiop:<host>:<port>/NameService"
  where host and port are, in this order of priority:
  - HPP_HOST, HPP_PORT environment variables
  - /hpp/host, /hpp/port ROS parameters
  - use default values ("localhost", 2809)
  """
  host = "localhost"
  port = "2809"
  import os
  try:
    import rospy
    # Check is ROS master is reachable.
    if rospy.client.get_master().target is not None:
      host = rospy.get_param("/hpp/host", host)
      port = rospy.get_param("/hpp/port", port)
  except:
    pass
  host = os.getenv ("HPP_HOST", host)
  port = os.getenv ("HPP_PORT", port)
  if host is None and port is None:
      url = "corbaloc:iiop:/NameService"
  else:
      url = "corbaloc:iiop:" \
            + (host if host is not None else "localhost") \
            + ":" + (port if port is not None else "2809") \
            + "/NameService"
  return url
