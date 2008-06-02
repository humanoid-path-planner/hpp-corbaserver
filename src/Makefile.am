# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

#
# Stand alone hppCorbaserver
#

bin_PROGRAMS = hppCorbaServer
hppCorbaServer_SOURCES = hppCorbaServer.cpp 

hppCorbaServer_CPPFLAGS = -I$(srcdir)/../include \
	@HPPCORE_CFLAGS@ \
	@OMNIORB4_CFLAGS@ 

hppCorbaServer_LDFLAGS = ./.libs/libhppCorbaServer.a \
	@HPPCORE_LIBS@ \
	@OMNIORB4_LIBS@ \
	@HRP2DYNAMICS_LIBS@

lib_LTLIBRARIES = libhppCorbaServer.la

EXTRA_DIST = \
	hppciExceptionHandlingMacros.h

BUILT_SOURCES = \
	hppciCommonServerSK.cc		\
	hppciObstacleServerSK.cc	\
	hppciProblemServerSK.cc		\
	hppciRobotServerSK.cc		\
	hppciCommonServer.hh		\
	hppciObstacleServer.hh		\
	hppciProblemServer.hh		\
	hppciRobotServer.hh 

nodist_libhppCorbaServer_la_SOURCES = \
	$(BUILT_SOURCES)

libhppCorbaServer_la_SOURCES= \
	hppciObstacle.cpp \
	hppciProblem.cpp \
	hppciRobot.cpp \
	hppciServer.cpp \
	hppciServerPrivate.cpp \
	hppciTools.cpp

libhppCorbaServer_la_CPPFLAGS = \
	-I@top_builddir@/include \
	-I$(srcdir)/../include \
	-I. \
	-I@top_builddir@/idl/server

libhppCorbaServer_la_CPPFLAGS += \
	@HPPCORE_CFLAGS@ \
	@OMNIORB4_CFLAGS@ \
	@WITH_OPENHRP@ 

libhppCorbaServer_la_LIBADD = \
	@HPPCORE_LIBS@ \
	@OMNIORB4_LIBS@ \
	@HRP2DYNAMICS_LIBS@

libhppCorbaServer_la_LDFLAGS= \
	-release $(PACKAGE_VERSION)

if OPENHRP
BUILT_SOURCES+= \
	common-modelloader.hh		\
	jrl-modelloader.hh

nodist_libhppCorbaServer_la_SOURCES+= \
	common-modelloader.hh	\
	jrl-modelloader.hh

libhppCorbaServer_la_SOURCES+= \
	hppciOpenHrp.cpp 

libhppCorbaServer_la_CPPFLAGS += \
	@HPPOPENHRP_CFLAGS@ -DSTRING_OPENHRP_PREFIX='"@OPENHRP_PREFIX@"' 

libhppCorbaServer_la_LIBADD += \
	@HPPOPENHRP_LIBS@

hppCorbaServer_CPPFLAGS += \
	@HPPOPENHRP_CFLAGS@ -DSTRING_OPENHRP_PREFIX='"@OPENHRP_PREFIX@"' 

hppCorbaServer_LDFLAGS += \
	@HPPOPENHRP_LIBS@

endif

clean-local:
	rm -f *SK.cc *.hh

%SK.cc %.hh: $(top_srcdir)/idl/server/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx $<

%SK.cc %.hh: $(top_builddir)/idl/server/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx -I$(top_srcdir)/idl/server $<

%SK.cc %.hh: @MODELLOADERJRL_IDL@/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx $<

# --- Kineo license ----------------------------------------------------

install-data-hook:
	-KPP_INSTALL_DIR=$(KINEOCONTROLLER_PREFIX) \
	LD_LIBRARY_PATH=$(KINEOCONTROLLER_PREFIX)/lib \
	$(KINEO_ADDON_BUILD) -a $(bindir)/hppCorbaServer
