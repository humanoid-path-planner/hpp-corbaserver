/** \mainpage 

\section hppCorbaServer_sec_intro Introduction

This package implements a Corba interface with hppCore package. Corba requests can be sent to trigger actions in a ChppPlanner object. 
Three main Corba interfaces are implemented:
\li ChppciRobot: to build a ChppDevice and to insert it in a ChppPlanner object,
\li ChppciObstacle: to build obstacles and insert them in a ChppPlanner object,
\li ChppciProblem: to define a path planning problem and solve it.

However, the main interface classes for users of this package are
\li ChppciServer that implements the above Corba interfaces
\li ChppciOpenHrpClient that implements a Corba client to load HRP2 model (installed on request see INSTALL file for instructions).

\section hppCorbaServer_sec_howto How to use this package

To use this package, create a ChppPlanner object specific to your problem.
\code
/*
  \brief This class implement my personal path planning algorithm for humanoid robot.
         It derives from ChppPlanner so that I can easily take advantage of HPP infrastructure
         (Corba server, KPP interface,...).
*/

class ChppSpecificPlanner : public ChppPlanner {
...
...
};

ChppSpecificPlanner* myHppPlanner = new ChppSpecificPlanner(...);
\endcode

Then construct a ChppciServer object with the above planner
\code
ChppciServer hppciServer(myHppPlanner);
\endcode

Then start the Corba server, passing arguments required to initialize Orb (in the following example, no argument is given).
\code
int argc=1;
char *argv[1] = {"NameOfMyProgram"};

hppciServer.startCorbaServer(argc, argv);
\endcode

The Corba server is now able to process external requests sent by other programs like python scripts for instance.
To enter in an infinite loop processing requests:
\code
hppciServer.processRequest(true);
\endcode
If you want only to process pending requests and return:
\code
hppciServer.processRequest(false);
\endcode


*/
