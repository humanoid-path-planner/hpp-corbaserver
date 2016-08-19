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

// #include <hpp/fcl/math/transform.h>
#include <pinocchio/spatial/se3.hpp>

using hpp::corbaServer::size_type;
using CORBA::ULong;

void
hppTransformToTransform3f
(const CORBA::Double* inConfig, hpp::corbaServer::Transform3f& transform)
{
  using hpp::corbaServer::Transform3f;
  Transform3f::Quaternion_t Q (inConfig [6], inConfig [3], inConfig [4], inConfig [5]);
  transform.translation() << inConfig [0], inConfig [1], inConfig [2];
  transform.rotation() = Q.matrix();
}

void
Transform3fTohppTransform (const hpp::corbaServer::Transform3f& transform,
			   CORBA::Double* config)
{
  using hpp::corbaServer::Transform3f;
  Transform3f::Quaternion_t q (transform.rotation());
  /*
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++)
      config.rot[i*3+j] = R (i,j);
  }
  */
  config[3] = q.x();
  config[4] = q.y();
  config[5] = q.z();
  config[6] = q.w();
  for(int i=0; i<3; i++)
    config [i] = transform.translation() [i];
}

hpp::floatSeq* vectorToFloatseq (const hpp::core::vector_t& input)
{
  CORBA::ULong size = (CORBA::ULong) input.size ();
  hpp::floatSeq* q_ptr = new hpp::floatSeq ();
  q_ptr->length (size);

  for (std::size_t i=0; i<size; ++i) {
    (*q_ptr) [(CORBA::ULong)i] = input [i];
  }
  return q_ptr;
}

hpp::floatSeqSeq* matrixToFloatSeqSeq (const hpp::core::matrix_t& input)
{
  hpp::floatSeqSeq* res = new hpp::floatSeqSeq;
  res->length ((ULong) input.rows ());
  for (size_type i=0; i<input.rows (); ++i) {
    hpp::floatSeq row; row.length ((ULong) input.cols ());
    for (size_type j=0; j<input.cols (); ++j) {
      row [(ULong) j] = input (i, j);
    }
    (*res) [(ULong) i] = row;
  }
  return res;
}

hpp::corbaServer::DevicePtr_t getRobotOrThrow (hpp::core::ProblemSolverPtr_t p)
{
  hpp::corbaServer::DevicePtr_t robot = p->robot ();
  if (!robot) throw hpp::Error ("No robot in problem solver.");
  return robot;
}
