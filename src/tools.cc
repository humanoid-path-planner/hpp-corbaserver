// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include "tools.hh"

#include <pinocchio/spatial/se3.hpp>

hpp::corbaServer::DevicePtr_t getRobotOrThrow (hpp::core::ProblemSolverPtr_t p)
{
  hpp::corbaServer::DevicePtr_t robot = p->robot ();
  if (!robot) throw hpp::Error ("No robot in problem solver.");
  return robot;
}
