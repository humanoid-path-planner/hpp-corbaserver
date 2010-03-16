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

void
ConfigurationToCkitMat4
(const hpp::Configuration inConfig, CkitMat4& outMatrix4)
{
  for(int i=0; i<3; i++)
    {
      for(int j=0; j<3; j++)
	outMatrix4(i,j) = inConfig.rot[i*3+j];
    }
  for(int i=0; i<3; i++)
    outMatrix4(i,3) = inConfig.trs[i];
  for(int i=0; i<3; i++)
    outMatrix4(3,i) = 0.0;
  outMatrix4(3,3) = 1.0;
}
