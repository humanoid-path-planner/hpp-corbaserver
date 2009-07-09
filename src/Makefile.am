include $(top_srcdir)/build-aux/init.mk
include $(top_srcdir)/build-aux/kineo.mk

BUILT_SOURCES =

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

SUFFIXES += SK.cc

%SK.cc %.hh: $(top_srcdir)/idl/server/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx $<

%SK.cc %.hh: $(top_builddir)/idl/server/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx -I$(top_srcdir)/idl/server $<

%SK.cc %.hh: @MODELLOADERJRL_IDL@/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx $<


lib_LTLIBRARIES = libhppCorbaServer.la

EXTRA_DIST += 				\
	hppciExceptionHandlingMacros.h

BUILT_SOURCES += 			\
	hppciCommonServerSK.cc		\
	hppciObstacleServerSK.cc	\
	hppciProblemServerSK.cc		\
	hppciRobotServerSK.cc		\
	hppciCommonServer.hh		\
	hppciObstacleServer.hh		\
	hppciProblemServer.hh		\
	hppciRobotServer.hh

nodist_libhppCorbaServer_la_SOURCES = 	\
	$(BUILT_SOURCES)

libhppCorbaServer_la_SOURCES = 		\
	hppciObstacle.cpp 		\
	hppciProblem.cpp 		\
	hppciRobot.cpp 			\
	hppciServer.cpp 		\
	hppciServerPrivate.cpp 		\
	hppciTools.cpp

libhppCorbaServer_la_CPPFLAGS = 	\
	-I$(top_builddir)/include 	\
	-I$(top_srcdir)/include 	\
	-I. 				\
	-I$(top_builddir)/idl/server	\
	@HPPCORE_CFLAGS@ 		\
	@OMNIORB4_CFLAGS@ 		\
	@WITH_OPENHRP@

libhppCorbaServer_la_LIBADD = 		\
	@HPPCORE_LIBS@ 			\
	@OMNIORB4_LIBS@ 		\
	@HRP2DYNAMICS_LIBS@

libhppCorbaServer_la_LDFLAGS = -release $(PACKAGE_VERSION)

 hppCorbaServer_CPPFLAGS =

if OPENHRP
 BUILT_SOURCES += 			\
	common-modelloader.hh		\
	jrl-modelloader.hh

 nodist_libhppCorbaServer_la_SOURCES += \
	common-modelloader.hh		\
	jrl-modelloader.hh

 libhppCorbaServer_la_SOURCES += 	\
	hppciOpenHrp.cpp

 libhppCorbaServer_la_CPPFLAGS += 	\
	@HPPOPENHRP_CFLAGS@ 		\
	$(AM_CPPFLAGS)			\
	-DSTRING_OPENHRP_PREFIX='"@OPENHRP_PREFIX@"'

 libhppCorbaServer_la_LIBADD += 	\
	@HPPOPENHRP_LIBS@

 hppCorbaServer_CPPFLAGS += 		\
	@HPPOPENHRP_CFLAGS@		\
	$(AM_CPPFLAGS)			\
	-DSTRING_OPENHRP_PREFIX='"@OPENHRP_PREFIX@"'

 hppCorbaServer_LDFLAGS = 		\
	@HPPOPENHRP_LIBS@

 CLEANFILES +=				\
	common-modelloaderSK.cc		\
	jrl-modelloaderSK.cc
endif


bin_PROGRAMS = hppCorbaServer
hppCorbaServer_SOURCES = hppCorbaServer.cpp

hppCorbaServer_CPPFLAGS +=		\
	$(AM_CPPFLAGS)			\
	@HPPCORE_CFLAGS@ 		\
	@OMNIORB4_CFLAGS@

hppCorbaServer_LDADD = 			\
	libhppCorbaServer.la		\
	@HPPCORE_LIBS@ 			\
	@OMNIORB4_LIBS@	 		\
	@HRP2DYNAMICS_LIBS@

BUILT_SOURCES += hppCorbaServer.kab



CLEANFILES += $(BUILT_SOURCES)
