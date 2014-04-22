// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <fcl/math/transform.h>
#include "tools.hh"

void
ConfigurationToTransform3f
(const hpp::Configuration inConfig, hpp::corbaServer::Transform3f& transform)
{
  fcl::Quaternion3f Q (inConfig.quat [0], inConfig.quat [1],
		     inConfig.quat [2], inConfig.quat [3]);
  fcl::Vec3f T (inConfig.trs [0], inConfig.trs [1], inConfig.trs [2]);
  transform.setTransform (Q, T);
}

void
Transform3fToConfiguration (const hpp::corbaServer::Transform3f& transform,
			    hpp::Configuration config)
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
    config.quat [i] = Q [i];
  }
  for(int i=0; i<3; i++)
    config.trs[i] = T [i];
}
