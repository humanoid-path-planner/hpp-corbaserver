# Don't expend macros beginning by JRL_.
m4_pattern_forbid([^JRL_])
AC_PREREQ([2.59])

## --------------------- ##
## Package configuration ##
## --------------------- ##
AC_INIT([hppCorbaServer],[1.7.4.99],[openrobot@laas.fr], [hppCorbaServer])

# Auxiliary files.
AC_CONFIG_AUX_DIR([build-aux])

# Automake.
AM_INIT_AUTOMAKE([1.9.6 dist-bzip2 -Wall nostdinc])

## ------------- ##
## C++ compiler  ##
## ------------- ##

# Search for a C++ compiler.
AC_LANG([C++])
AC_PROG_CXX

# Speed-up compilation.
if test "$GXX" = yes; then
  CXXFLAGS="$CXXFLAGS -pipe"
fi

# Macros are stored in build-aux.
AC_CONFIG_MACRO_DIR([build-aux])

## ------------------- ##
## Additionnal tools.  ##
## ------------------- ##

# Search for misc. common tools.
AC_PROG_MKDIR_P
AC_PROG_SED
AC_PROG_INSTALL

# Customize warnings.
JRL_CXX_WARNINGS([[-Wall],
                   [-Wcast-align],
                   [-Wcast-qual],
                   [-Wformat],
                   [-Wmissing-prototypes],
                   [-Wstrict-prototypes],
                   [-Wwrite-strings],
                   [-Wbad-function-cast],
                   [-Wmissing-declarations],
                   [-Wnested-externs]])

# Search for Libtool.
LT_INIT([dlopen])

# Search for Doxygen.
JRL_PROG_DOXYGEN

## ------------------------- ##
## Search for dependencies.  ##
## ------------------------- ##

PKG_CHECK_MODULES([KINEOCONTROLLER], [KineoController])
KINEOCONTROLLER_PREFIX=`$PKG_CONFIG KineoPathPlanner --variable=prefix`

AC_SUBST([KINEOCONTROLLER_PREFIX])
AC_SUBST([KINEO_ADDON_BUILDER],
[${KINEOCONTROLLER_PREFIX}/bin/KineoAddonBuilder])
AC_SUBST([KINEO_PREFIX], [${KINEOCONTROLLER_PREFIX}])
AC_SUBST([KINEO_LIBDIR], [${KINEOCONTROLLER_PREFIX}/lib])

define(kwsPlus_reqd, [kwsPlus >= 1.7])
AC_SUBST([KWSPLUS_REQD], ["kwsPlus_reqd"])
PKG_CHECK_MODULES([KWSPLUS], [kwsPlus_reqd])
KWSPLUS_PREFIX=`$PKG_CONFIG kwsPlus --variable=prefix`
AC_SUBST([KWSPLUS_PREFIX])

define(hppcore_reqd, [hppCore >= 1.8.1])
AC_SUBST([HPPCORE_REQD], ["hppcore_reqd"])
PKG_CHECK_MODULES([HPPCORE], [hppcore_reqd])
HPPCORE_DOCDIR=`$PKG_CONFIG hppCore --variable=docdir`
AC_SUBST([HPPCORE_DOCDIR])

PKG_CHECK_MODULES([OMNIORB4], [omniORB4])
OMNIORB_PREFIX=`$PKG_CONFIG omniORB4 --variable=prefix`
AC_SUBST([OMNIORB_PREFIX])

AC_SUBST([WITH_OPENHRP])
AC_ARG_WITH([openhrp],
	[AS_HELP_STRING([--with-openhrp], [to implement an OpenHRP client])],
	AC_SUBST([WITH_OPENHRP], ["-DWITH_OPENHRP=1"])
	AC_SUBST([OPENHRP], ["1"])
	with_openhrp=true,
	AC_SUBST([WITH_OPENHRP], ["-DWITH_OPENHRP=0"])
	AC_SUBST([OPENHRP], ["0"])
	with_openhrp=false
)
AM_CONDITIONAL([OPENHRP], [test x$with_openhrp = xtrue])

AC_SUBST([OPENHRP_DEPENDENCES], [])

if test x$with_openhrp = xtrue; then
	define(hppopenhrp_reqd, [hppOpenHRP >= 1.6.1])
	AC_SUBST([HPPOPENHRP_REQD], ["hppopenhrp_reqd"])
	PKG_CHECK_MODULES([HPPOPENHRP], [hppopenhrp_reqd])
	HPPOPENHRP_PREFIX=`$PKG_CONFIG hppOpenHRP --variable=prefix`
	OPENHRP_PREFIX=`$PKG_CONFIG hppOpenHRP --variable=OpenHRP_prefix`
	AC_SUBST([HPPOPENHRP_PREFIX])
	AC_SUBST([OPENHRP_PREFIX])

	define(jrlmodelloader_reqd, [jrl-modelloader >= 1.4])
	AC_SUBST([JRLMODELLOADER_REQD], ["jrlmodelloader_reqd"])
	PKG_CHECK_MODULES([MODELLOADERJRL], [jrlmodelloader_reqd])
	MODELLOADERJRL_IDL=`$PKG_CONFIG jrl-modelloader --variable=idldir`
	AC_SUBST([MODELLOADERJRL_IDL])

	define(hrp2dynamics_reqd, [hrp2Dynamics >= 1.1])
	AC_SUBST([HRP2DYNAMICS_REQD], ["hrp2dynamics_reqd"])
	PKG_CHECK_MODULES([HRP2DYNAMICS], [hrp2dynamics_reqd])

	define(hrp2_14_reqd, [hrp2_14 >= 1.6.1])
	AC_SUBST([HRP2_14_REQD], ["hrp2_14_reqd"])
	PKG_CHECK_MODULES([HRP2_14], [hrp2_14_reqd])

	OPENHRP_DEPENDENCES="hppopenhrp_reqd, jrlmodelloader_reqd, hrp2dynamics_reqd, hrp2_14_reqd"

	AC_SUBST([DISTCHECK_CONFIGURE_FLAGS], [--with-openhrp])
fi


## ------------ ##
## Pkg-config.  ##
## ------------ ##

AC_SUBST([PKGCONFIG_DESCRIPTION], ['Corba server of humanoid path planner'])
AC_SUBST([PKGCONFIG_URL], [])
AC_SUBST([PKGCONFIG_REQUIRES],
	 ["${HPPCORE_REQD}, omniORB4 >= 4.0.7, ${OPENHRP_DEPENDENCES}"])
AC_SUBST([PKGCONFIG_CONFLICTS], [-lhppCorbaServer])
AC_SUBST([PKGCONFIG_LIBS], [-lhppCore])
AC_SUBST([PKGCONFIG_CFLAGS], [])
AC_SUBST([PKGCONFIG_EXTRA], [openhrp_flag=${WITH_OPENHRP}])

## -------------- ##
## Write output.  ##
## -------------- ##

# Create config.h
AC_CONFIG_HEADERS([config.h])

# Creating Makefiles.
AC_OUTPUT([
	Makefile
	 idl/server/Makefile
	 idl/server/hppciRobotServer.idl
	 idl/server/hppciObstacleServer.idl
	include/Makefile
	 include/hppCorbaServer/hppciOpenHrp.h
	src/Makefile
	doc/Makefile
])
