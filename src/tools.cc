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
  fcl::Matrix3f R;
  fcl::Vec3f T;
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++)
      R (i,j) = inConfig.rot[i*3+j];
  }
  for(int i=0; i<3; i++)
    T [i] = inConfig.trs[i];
  transform.setTransform (R, T);
}
