dnl autoconf script for hppCorbaServer libs
dnl
dnl   Research carried out within the scope of the Associated International 
dnl   Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux and Mathieu Poirier LAAS-CNRS
dnl

AC_INIT([hppCorbaServer],[1.0],[openrobot@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

PKG_CHECK_MODULES(HPPCORE, hppCore)
HPPCORE_PREFIX=`$PKG_CONFIG hppCore --variable=prefix`
AC_SUBST(HPPCORE_CFLAGS)
AC_SUBST(HPPCORE_LIBS)
AC_SUBST(HPPCORE_PREFIX)

PKG_CHECK_MODULES(OMNIORB4, omniORB4)
OMNIORB_PREFIX=`$PKG_CONFIG omniORB4 --variable=prefix`
AC_SUBST(OMNIORB4_CFLAGS)
AC_SUBST(OMNIORB4_LIBS)
AC_SUBST(OMNIORB_PREFIX)

AC_SUBST(WITH_OPENHRP)
AC_ARG_WITH(openhrp,
	AS_HELP_STRING([--with-openhrp], [to implemente an OpenHRP client]),
	CXXFLAGS="${CXXFLAGS} -DWITH_OPENHRP=1"
	with_openhrp=true,
	CXXFLAGS="${CXXFLAGS} -DWITH_OPENHRP=0"
	with_openhrp=false
)
AM_CONDITIONAL(OPENHRP, test x$with_openhrp = xtrue)

AC_SUBST(OPENHRP_DEPENDENCES, [])

if test x$with_openhrp = xtrue; then
	PKG_CHECK_MODULES(HPPOPENHRP, hppOpenHRP)
	HPPOPENHRP_PREFIX=`$PKG_CONFIG hppOpenHRP --variable=prefix`
	OPENHRP_PREFIX=`$PKG_CONFIG hppOpenHRP --variable=OpenHRP_prefix`
	AC_SUBST(HPPOPENHRP_CFLAGS)
	AC_SUBST(HPPOPENHRP_LIBS)
	AC_SUBST(HPPOPENHRP_PREFIX)
	AC_SUBST(OPENHRP_PREFIX)
	
	PKG_CHECK_MODULES(MODELLOADERJRL, jrl-modelloader)
	MODELLOADERJRL_IDL=`$PKG_CONFIG jrl-modelloader --variable=idldir`
	AC_SUBST(MODELLOADERJRL_IDL)

	OPENHRP_DEPENDENCES=", hppOpenHRP >= 1.0, jrl-modelloader >= 1.0"
fi

AC_OUTPUT(Makefile
	idl/server/Makefile
	src/Makefile
	include/Makefile
	doc/Makefile
	doc/hpp.dox
	hppCorbaServer.pc
)


