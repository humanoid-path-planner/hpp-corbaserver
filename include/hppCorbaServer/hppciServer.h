/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#ifndef HPPCI_SERVER_H
#define HPPCI_SERVER_H

#include "hppCore/hppPlanner.h"

class ChppciServerPrivate;

/** 

The Corba server of Library Hpp is mainly implemented by class ChppciServer. This class basically:
\li initializes a Corba server that implements several interfaces defined by the following modules,
\li instantiate an object ChppPlanner a pointer to which is stored in ChppciServer::hppPlanner.
The Corba interfaces trigger actions processed by ChppPlanner object. More information is provided in each interface documentation.
 */

/** \brief Implementation of Hpp module Corba server.
 *
This class initializes the Corba server and starts the following Corba interface implementations.
\li ChppciServer::robotServant,
\li ChppciServer::obstacleServant,
\li ChppciServer::problemServant.

To use this object, call the constructor
\code
int argc=1;
char *argv[1] = {"program"};
ChppPlanner* hppPlanner = new ChppPlanner;
ChppciServer server(ChppPlanner* hppPlanner, argc, argv);
\endcode
After starting a name server and configuring your Corba implementation, start the servers.
\code
server.startCorbaServer();
\endcode
Then, enter in the loop that handle the Corba requests
\code
server.processRequest(true);
\endcode
You can then send request to the servers.
*/

class ChppciServer {
public:
  /** 
      \brief Constructor 
      \param inHppPlanner the object that will handle Corba requests.
      \param argc, argv parameter to feed ORB initialization. 
      
      \note It is recommended to configure your Corba implementation through environment variables and to set argc to 1 and argv to any string.
  */
  ChppciServer(ChppPlanner* inHppPlanner, int argc, char *argv[]);
  /// \brief Shutdown CORBA server
  ~ChppciServer();
  /// \brief Initialize CORBA server to process requests from clients to hpp module
  /// \return 0 if success, -1 if failure.
  int startCorbaServer();
  /// \brief If ORB work is pending, process it
  /// \param loop if true, the function never returns; if false, the function processes pending requests and returns.
  int processRequest(bool loop);
  /// \brief return a pointer to object ChppciServer::hppPlanner.
  ChppPlanner *getHppPlanner();

  // 
  // Static public
  // 
  static ChppciServer* getInstance();

private:

  ktStatus initORBandServers(int argc, char *argv[]);

  ChppciServerPrivate* attPrivate;
  /// \brief static pointer to the only object of this class.
  static ChppciServer* s_hppciServer;

  /// \brief pointer to ChppPlanner Object.
  /// At initialization, the constructor creates a ChppPlanner object and keeps a pointer to it. All Corba requests are processed by this object. Notice that this pointer is passed to each constructor of implementation classes of the server Corba interface.
  ChppPlanner *hppPlanner;

};

#endif
