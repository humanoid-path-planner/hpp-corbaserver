// Copyright (C) 2010 by Thomas Moulard, CNRS.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_FWD_HH
# define HPP_CORBASERVER_FWD_HH
# include <KineoUtility/kitDefine.h>

//FIXME: should be replaced by CORBA base types forward declarations.
# include <omniORB4/CORBA.h>

KIT_PREDEF_CLASS(CkwsDiffusionNodePicker);
KIT_PREDEF_CLASS(CkwsDiffusionShooter);

class CkwsPlusSteeringMethodFactory;
class CkwsPlusDistanceFactory;
class CkwsPlusDiffusionNodePickerFactory;
class CkwsPlusDiffusionShooterFactory;

namespace hpp
{
  namespace corbaServer
  {
    class Server;

    namespace impl
    {
      using CORBA::Boolean;
      using CORBA::Double;
      using CORBA::Short;
      using CORBA::SystemException;
      using CORBA::ULong;
      using CORBA::UShort;

      class Problem;
      class Obstacle;
      class Robot;
      class Server;
    }
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif //! HPP_CORBASERVER_FWD_HH
