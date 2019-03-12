/** \mainpage 
\anchor hpp_corbaserver_documentation

\section hppCorbaServer_sec_intro Introduction

This package implements a Corba interface with hppCore package. Corba requests can be sent to trigger actions in a hpp::core::ProblemSolver object. 
Three main Corba interfaces are implemented:
\li hpp::corbaserver::Robot: to build a hpp::model::Device and to insert it in a hpp::core::ProblemSolver object,
\li hpp::corbaserver::Obstacle: to build obstacles and insert them in a hpp::core::ProblemSolver object,
\li hpp::corbaserver::Problem: to define a path planning problem and solve it.

However, the main interface classes for users of this package are
\li hpp::corbaServer::Server that implements the above Corba interfaces

\section hppCorbaServer_sec_howto How to communicate with the CORBA server

The easiest way is
\li to launch \c hppcorbaserver executable or any executable that implements the server,
\li open a python terminal and type:
\code
from hpp.corbaserver import Client
cl = Client ()
\endcode
Then variable \c cl contains three members \c robot, \c obstacle and \c problem that can send request to the server.

Some python classes are provided that embed corba clients and
that forward corba resquests to the server side:
  \li hpp.corbaserver.robot.Robot to load and handle a robot,
  \li hpp.corbaserver.robot.HumanoidRobot to load and handle a humanoid robot,
  \li hpp.corbaserver.problem_solver.ProblemSolver to set and solve a path planning problem.
  \li hpp.corbaserver.benchmark.Benchmark to generate statistics on solving time and length of solutions.

\section hppCorbaServer_sec_embedding How to embed a server in an application

To embed a CORBA server in an application, see documentation of class hpp::corbaServer::Server.

*/
