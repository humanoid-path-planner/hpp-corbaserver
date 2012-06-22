// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPPCORBASERVER_TOOLS_HH
# define HPPCORBASERVER_TOOLS_HH
# include <KineoUtility/kitMat4.h>
# include <hpp/util/portability.hh>

# include <hpp/corbaserver/config.hh>
# include "common.hh"

HPP_CORBASERVER_LOCAL void
ConfigurationToCkitMat4 (const hpp::Configuration inConfig,
			 CkitMat4& outMatrix4);

#endif //! HPPCORBASERVER_TOOLS_HH
