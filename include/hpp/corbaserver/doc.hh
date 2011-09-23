/** \mainpage 

\section hppCorbaServer_sec_intro Introduction

This package implements a Corba interface with hppCore package. Corba requests can be sent to trigger actions in a hpp::core::Planner object. 
Three main Corba interfaces are implemented:
\li hpp::Robot: to build a hpp::model::Device and to insert it in a hpp::core::Planner object,
\li hpp::Obstacle: to build obstacles and insert them in a hpp::core::Planner object,
\li hpp::Problem: to define a path planning problem and solve it.

However, the main interface classes for users of this package are
\li hpp::corbaServer::Server that implements the above Corba interfaces
\li hpp::corbaServer::impl::OpenHRP that implements a Corba client to load HRP2 model (installed on request see INSTALL file for instructions).

\section hppCorbaServer_sec_howto How to use this package

To use this package, See documentation of class hpp::corbaServer::Server.

*/
