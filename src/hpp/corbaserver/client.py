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

  defaultClients = {
          'problem' : hpp.corbaserver.Problem,
          'obstacle': hpp.corbaserver.Obstacle,
          'robot'   : hpp.corbaserver.RobotIDL,
          }

  def makeClient(self, serviceName, class_, postContextId):
    """
    Create a client to a new CORBA service and add it to this class.
    """
    serviceName = serviceName.lower ()
    name = [CosNaming.NameComponent ("hpp" + postContextId, "corbaserver"),
            CosNaming.NameComponent ("basic", serviceName)]

    try:
      obj = self.rootContext.resolve (name)
    except CosNaming.NamingContext.NotFound, ex:
      raise CorbaError (
        'failed to find the service ``{0}\'\''.format (serviceName))

    try:
      client = obj._narrow (class_)
    except KeyError:
      raise CorbaError ('invalid service name ``{0}\'\''.format (serviceName))

    if client is None:
      # This happens when stubs from client and server are not synchronized.
      raise CorbaError (
        'failed to narrow client for service named ``{0}\'\''.format
        (serviceName))
    self.__dict__[serviceName] = client


  def __init__(self, clients = defaultClients, url = None, postContextId=""):
    """
    Initialize CORBA and create default clients.
    :param url: URL in the IOR, corbaloc, corbalocs, and corbanames formats.
                For a remote corba server, use
                url = "corbaloc:iiop:<host>:<port>/NameService"
    """
    import sys
    self.orb = CORBA.ORB_init (sys.argv, CORBA.ORB_ID)
    if url is None:
        obj = self.orb.string_to_object (_getIIOPurl())
    else:
        obj = self.orb.string_to_object (url)
    self.rootContext = obj._narrow(CosNaming.NamingContext)
    if self.rootContext is None:
      raise CorbaError ('failed to narrow the root context')

    for client, class_ in clients.iteritems():
      self.makeClient (client, class_, postContextId)

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
