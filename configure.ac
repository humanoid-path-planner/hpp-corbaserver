# Don't expend macros beginning by JRL_.
m4_pattern_forbid([^JRL_])
AC_PREREQ([2.63])

## --------------------- ##
## Package configuration ##
## --------------------- ##
AC_INIT([hpp-corbaserver],
        [1.16],
	[hpp@laas.fr],
	[hpp-corbaserver])

# Auxiliary files.
AC_CONFIG_AUX_DIR([build-aux])

# Automake.
AM_INIT_AUTOMAKE([1.9.6 dist-bzip2 -Wall nostdinc])

# Check configure is called properly.
AC_CONFIG_SRCDIR([src/hpp-corbaserver.cc])

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

# Customize warnings.
if  test "$GXX" = yes; then
  CXXFLAGS="$CXXFLAGS"
  CXXFLAGS="$CXXFLAGS -Wall"
  CXXFLAGS="$CXXFLAGS -Wcast-align"
  CXXFLAGS="$CXXFLAGS -Wcast-qual"
  CXXFLAGS="$CXXFLAGS -Wformat"
  CXXFLAGS="$CXXFLAGS -Wwrite-strings"
  CXXFLAGS="$CXXFLAGS -Wmissing-declarations"
fi

# Macros are stored in build-aux.
AC_CONFIG_MACRO_DIR([build-aux])

## --------------------------------------------- ##
## Optionally enable OpenHRP integration         ##
## --------------------------------------------- ##
AC_ARG_ENABLE([openhrp],
[AS_HELP_STRING([--enable-openhrp],
 [enable OpenHRP (disabled by default)])],
[AC_DEFINE([HPP_CORBASERVER_ENABLE_OPENHRP], [1],
           [Define to 1 if OpenHRP is enabled.])
enable_openhrp=true])
AM_CONDITIONAL([OPENHRP], [test x$enable_openhrp = xtrue])

## ------------------- ##
## Additionnal tools.  ##
## ------------------- ##

# Search for misc. common tools.
AC_PROG_MKDIR_P
AC_PROG_SED

# Search for omniidl.
AC_ARG_VAR([OMNIIDL], [the omniORB idl compiler])
AC_CHECK_PROGS([OMNIIDL], [omniidl])
if test x"$OMNIIDL" = x; then
 AC_MSG_ERROR([failed to find omniidl])
fi

# Search for Doxygen.
JRL_PROG_DOXYGEN

# Search for Libtool.
LT_INIT

# Search for pkg-config dependencies.
JRL_PKGCONFIGCHECK_ADDDEP([HPPCORE], [hpp-core >= 2.2])
JRL_PKGCONFIGCHECK_ADDDEP([HPP_UTIL], [hpp-util >= 0.4])
JRL_PKGCONFIGCHECK([KWSPLUS], [kwsPlus >= 2.1])
JRL_PKGCONFIGCHECK_ADDDEP([OMNIORB4], [omniORB4])

JRL_PKGCONFIGCHECK([KINEOCONTROLLER], [KineoController >= 2.06])
AC_SUBST([KINEO_LIBDIR],[${KINEOCONTROLLER_PREFIX}/lib])
AC_SUBST(KINEO_PREFIX, [${KINEOPATHPLANNER_PREFIX}])
AC_SUBST([KINEO_ADDON_BUILDER],
	[${KINEOCONTROLLER_PREFIX}/bin/KineoAddonBuilder])

# Additional dependencies when OpenHRP is enabled.
if test x$enable_openhrp = xtrue; then
   JRL_PKGCONFIGCHECK_ADDDEP([HPPOPENHRP], [hppOpenHRP >= 1.9])
   AC_SUBST([OPENHRP_PREFIX],
            [`$PKG_CONFIG hppOpenHRP --variable=OpenHRP_prefix`])
   JRL_PKGCONFIGCHECK_ADDDEP([JRLMODELLOADER], [jrl-modelloader >= 1.4])
   AC_SUBST([MODELLOADERJRL_IDL],
            [`$PKG_CONFIG jrl-modelloader --variable=idldir`])
   JRL_PKGCONFIGCHECK_ADDDEP([HRP2DYNAMICS], [hrp2-dynamics >= 1.5])
   JRL_PKGCONFIGCHECK_ADDDEP([HRP2_14], [hrp2-14 >= 1.8])
   JRL_PKGCONFIGCHECK_ADDDEP([ROBOTBUILDER], [robotbuilder >= 1.2])
fi


# Generate ``config.hh'' header file.
AC_SUBST([PACKAGE_CPPNAME],
 [`echo "$PACKAGE_TARNAME" \
  | tr '@<:@:lower:@:>@' '@<:@:upper:@:>@' | sed 's/-/_/g'`])
AC_CONFIG_FILES([include/hpp/corbaserver/config.hh])

## ------------ ##
## Test-suite.  ##
## ------------ ##

AC_CONFIG_TESTDIR([tests])
AM_MISSING_PROG([AUTOM4TE], [autom4te])

## ------------ ##
## Pkg-config.  ##
## ------------ ##

AC_SUBST([PKGCONFIG_DESCRIPTION], ['HPP CORBA server'])
AC_SUBST([PKGCONFIG_URL], [])
AC_SUBST([PKGCONFIG_REQUIRES], [`echo "$PKGCONFIG_REQUIRES" | sed 's/, $//'`])
AC_SUBST([PKGCONFIG_CONFLICTS], [])
AC_SUBST([PKGCONFIG_LIBS], [-lhpp-corbaserver])
AC_SUBST([PKGCONFIG_CFLAGS], [])


## -------------- ##
## Write output.  ##
## -------------- ##

# Create config.h
AC_CONFIG_HEADERS([config.h])

# Create Makefiles.
AC_CONFIG_FILES([
  Makefile
  idl/Makefile
  idl/hpp/corbaserver/Makefile
  include/Makefile
  doc/Makefile
  src/Makefile
  tests/Makefile
  tests/atlocal
  tests/package.m4
])

if test x$enable_openhrp = xtrue; then
   AC_CONFIG_FILES([
	include/hpp/corbaserver/openhrp.hh
	])
fi

# Write files.
AC_OUTPUT
