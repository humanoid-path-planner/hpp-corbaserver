// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <hpp/fcl/math/transform.h>
#include "tools.hh"

void
hppTransformToTransform3f
(const CORBA::Double* inConfig, hpp::corbaServer::Transform3f& transform)
{
  fcl::Quaternion3f Q (inConfig [3], inConfig [4],
		       inConfig [5], inConfig [6]);
  fcl::Vec3f T (inConfig [0], inConfig [1], inConfig [2]);
  transform.setTransform (Q, T);
}

void
Transform3fTohppTransform (const hpp::corbaServer::Transform3f& transform,
			   CORBA::Double* config)
{
  fcl::Quaternion3f Q = transform.getQuatRotation ();
  fcl::Vec3f T = transform.getTranslation ();
  /*
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++)
      config.rot[i*3+j] = R (i,j);
  }
  */
  for(int i=0; i<4; i++) {
    config [i+3] = Q [i];
  }
  for(int i=0; i<3; i++)
    config [i] = T [i];
}
