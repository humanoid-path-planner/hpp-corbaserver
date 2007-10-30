# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

lib_LTLIBRARIES = libhppCorbaServer.la

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
	@OMNIORB4_LIBS@ 

libhppCorbaServer_la_LDLAGS= \
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
endif

clean-local:
	rm -f *SK.cc *.hh

%SK.cc %.hh: ../idl/server/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx @WITH_OPENHRP@ $<

%SK.cc %.hh: @MODELLOADERJRL_IDL@/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx $<
