/** \mainpage 

\section hppCorbaServer_sec_intro Introduction

This package implements a Corba interface with hppCore package. Corba requests can be sent to trigger actions in a ChppPlanner object. 
Three main Corba interfaces are implemented:
\li hppCorbaServer::ChppciRobot: to build a ChppDevice and to insert it in a ChppPlanner object,
\li hppCorbaServer::ChppciObstacle: to build obstacles and insert them in a ChppPlanner object,
\li hppCorbaServer::ChppciProblem: to define a path planning problem and solve it.

However, the main interface classes for users of this package are
\li ChppciServer that implements the above Corba interfaces
\li ChppciOpenHrpClient that implements a Corba client to load HRP2 model (installed on request see INSTALL file for instructions).

\section hppCorbaServer_sec_howto How to use this package

To use this package, See documentation of class ChppciServer.

*/
