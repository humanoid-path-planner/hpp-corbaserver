dnl autoconf script for hppCorbaServer libs
dnl
dnl   Research carried out within the scope of the Associated International 
dnl   Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux and Mathieu Poirier LAAS-CNRS
dnl

AC_INIT([hppCorbaServer],[1.5],[openrobot@laas.fr])
AC_PREREQ(2.59)

if test "$CFLAGS$CXXFLAGS" = ""; then
  setdefaultflags=yes
else
  setdefaultflags=no
fi

AM_INIT_AUTOMAKE([foreign no-define])
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

if test "$setdefaultflags" = "yes"; then
  if test "$GCC" = "yes"; then
    CFLAGS="$CFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c flags])
  fi
  if test "$GXX" = "yes"; then
    CXXFLAGS="$CXXFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c++ flags])
  fi
fi

PKG_CHECK_MODULES(KINEOCONTROLLER, KineoController)
KINEOCONTROLLER_PREFIX=`$PKG_CONFIG KineoPathPlanner --variable=prefix`
AC_SUBST(KINEOCONTROLLER_PREFIX)
AC_SUBST(KINEO_ADDON_BUILD,[${KINEOCONTROLLER_PREFIX}/bin/KineoAddonBuilder])

define(kwsPlus_reqd, [kwsPlus >= 1.5])
AC_SUBST(KWSPLUS_REQD, "kwsPlus_reqd")
PKG_CHECK_MODULES(KWSPLUS, kwsPlus_reqd)
KWSPLUS_PREFIX=`$PKG_CONFIG kwsPlus --variable=prefix`
AC_SUBST(KWSPLUS_PREFIX)

define(hppcore_reqd, [hppCore >= 1.5])
AC_SUBST(HPPCORE_REQD, "hppcore_reqd")
PKG_CHECK_MODULES(HPPCORE, hppcore_reqd)
HPPCORE_PREFIX=`$PKG_CONFIG hppCore --variable=prefix`
AC_SUBST(HPPCORE_PREFIX)

PKG_CHECK_MODULES(OMNIORB4, omniORB4)
OMNIORB_PREFIX=`$PKG_CONFIG omniORB4 --variable=prefix`
AC_SUBST(OMNIORB_PREFIX)

AC_SUBST(WITH_OPENHRP)
AC_ARG_WITH(openhrp,
	AS_HELP_STRING([--with-openhrp], [to implement an OpenHRP client]),
	AC_SUBST(WITH_OPENHRP, "-DWITH_OPENHRP=1")
	AC_SUBST(OPENHRP, "1")
	with_openhrp=true,
	AC_SUBST(WITH_OPENHRP, "-DWITH_OPENHRP=0")
	AC_SUBST(OPENHRP, "0")
	with_openhrp=false
)
AM_CONDITIONAL(OPENHRP, test x$with_openhrp = xtrue)

AC_SUBST(OPENHRP_DEPENDENCES, [])

if test x$with_openhrp = xtrue; then
	define(hppopenhrp_reqd, [hppOpenHRP >= 1.2])
	AC_SUBST(HPPOPENHRP_REQD, "hppopenhrp_reqd")
	PKG_CHECK_MODULES(HPPOPENHRP, hppopenhrp_reqd)
	HPPOPENHRP_PREFIX=`$PKG_CONFIG hppOpenHRP --variable=prefix`
	OPENHRP_PREFIX=`$PKG_CONFIG hppOpenHRP --variable=OpenHRP_prefix`
	AC_SUBST(HPPOPENHRP_PREFIX)
	AC_SUBST(OPENHRP_PREFIX)
	
	define(jrlmodelloader_reqd, [jrl-modelloader >= 1.2])
	AC_SUBST(JRLMODELLOADER_REQD, "jrlmodelloader_reqd")
	PKG_CHECK_MODULES(MODELLOADERJRL, jrlmodelloader_reqd)
	MODELLOADERJRL_IDL=`$PKG_CONFIG jrl-modelloader --variable=idldir`
	AC_SUBST(MODELLOADERJRL_IDL)

	define(hrp2dynamics_reqd, [hrp2Dynamics >= 1.0])
	AC_SUBST(HRP2DYNAMICS_REQD, "hrp2dynamics_reqd")
	PKG_CHECK_MODULES(HRP2DYNAMICS, hrp2dynamics_reqd)

	define(hrp2_14_reqd, [hrp2_14 >= 1.0])
	AC_SUBST(HRP2_14_REQD, "hrp2_14_reqd")
	PKG_CHECK_MODULES(HRP2_14, hrp2_14_reqd)

	OPENHRP_DEPENDENCES=", hppopenhrp_reqd, jrlmodelloader_reqd, hrp2dynamics_reqd, hrp2_14_reqd"
fi

AC_OUTPUT(Makefile
	idl/server/Makefile
	idl/server/hppciRobotServer.idl
	idl/server/hppciObstacleServer.idl
	src/Makefile
	include/Makefile
	include/hppCorbaServer/hppciOpenHrp.h
	doc/Makefile
	doc/hpp.dox
	hppCorbaServer.pc
)


