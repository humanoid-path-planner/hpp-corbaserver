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

\page hpp_corbaserver_bind_cpp_objects Bind C++ objects automatically

\par Module hierarchy.
It should respect the C++ hierarchy. At the moment, the topmost
module is \c hpp. The other module should either be as in C++ or suffixed with \c _idl.
Suffixing is usefull to prevent name collisions.

\par IDL comments prefix
They are used to trigger particular behavior.
- '// ' is a standard comment (two slashes followed by one space).
- '//*' replaces the automatic implementation by the code provided. All
        consecutive lines starting with this prefix will be used.

        You also find them after the hpp module. The code in this context will
        be put at the beginning of the generated C++ file. This is useful to
        include C++ headers.
- '//->' is used to rename a function, a module or an interface (the last may not be implemented yet).
- '///' is a comment used to generate the documentation (by Python and Doxygen).

\par What to do when the automatic implementation does not work ?

1. The compiler should tell you what function fails.
2. Understand why this function fails. You can modify the file in the build folder.
3. When your changes work as expected, copy the code in a *comment block for code*
   (i.e. comments starts with //*) just after the function declaration.
   No need to include the try/catch block.

As an example, take function DifferentiableFunction.value, in
idl/hpp/constraints_idl/constraints.idl.
The automatic implementation for this function cannot work since LieGroupElement
cannot be converted (at least, as of now). So you must provide a implementation.
Have a look at the comment in the file.

\par Use CMake to generate the C++ implementation of your IDL.
The best is to have a look at src/CMakeLists.txt. Eventually,
hpp-manipulation-corba/src/CMakeLists.txt may be informative.
Most of the complexity is hidden behind the CMake macros
\c GENERATE_IDL_PYTHON, \c GENERATE_IDL_CPP and \c GENERATE_IDL_CPP_IMPL.

If you encounter an error related to a module not found, it may be because you
forgot an option *-Wbextern=...* in the CMake function GENERATE_IDL_PYTHON

\todo The fact that one must specify what are the external module, for the Python
client, is very annoying. One workaround could be to generate only one big Python
file instead of plenty of separated ones. To do this, it should be sufficient to
pass multiple IDL files to omniidl.

*/
