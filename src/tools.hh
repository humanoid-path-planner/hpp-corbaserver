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

# include <hpp/corbaserver/config.hh>
# include <hpp/corbaserver/fwd.hh>
# include "common.hh"

HPP_CORBASERVER_LOCAL void
ConfigurationToTransform3f (const hpp::Configuration inConfig,
			    hpp::corbaServer::Transform3f& outMatrix4);

#endif //! HPPCORBASERVER_TOOLS_HH
