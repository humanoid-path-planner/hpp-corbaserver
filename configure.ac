# Don't expend macros beginning by JRL_.
m4_pattern_forbid([^JRL_])
AC_PREREQ([2.59])

## --------------------- ##
## Package configuration ##
## --------------------- ##
AC_INIT([hppCorbaServer],[1.8.2.99],[openrobot@laas.fr], [hppCorbaServer])

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

# Search for pkg-config dependencies.
m4_include([deps.ac])

## ------------ ##
## Pkg-config.  ##
## ------------ ##

AC_SUBST([PKGCONFIG_DESCRIPTION], ['Corba server of humanoid path planner'])
AC_SUBST([PKGCONFIG_URL], [])
AC_SUBST([PKGCONFIG_REQUIRES],
	 ["${HPPCORE_REQD}, omniORB4 >= 4.0.7${OPENHRP_DEPENDENCES}"])
AC_SUBST([PKGCONFIG_CONFLICTS], [])
AC_SUBST([PKGCONFIG_LIBS], [-lhppCorbaServer])
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
