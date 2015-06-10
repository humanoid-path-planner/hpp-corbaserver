// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>
#include <hpp/util/debug.hh>
#include "hpp/corbaserver/server.hh"

using hpp::corbaServer::Server;

int
main (int argc, const char* argv[])
{
  hpp::core::ProblemSolverPtr_t problemSolver =
    hpp::core::ProblemSolver::create ();
  Server server (problemSolver, argc, argv, true);

  server.startCorbaServer ();
  server.processRequest(true);
}
