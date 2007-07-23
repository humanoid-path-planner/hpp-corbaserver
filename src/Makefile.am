# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

lib_LTLIBRARIES = libhppCorbaServer.la

libhppCorbaServer_la_SOURCES = \
	hppciCommonServerSK.cc \
	hppciObstacleServerSK.cc \
	hppciProblemServerSK.cc \
	hppciRobotServerSK.cc 

libhppCorbaServer_la_SOURCES += \
	common-modelloaderSK.cc \
	jrl-modelloaderSK.cc \
	hppciOpenHrp.cpp \
	hppciObstacle.cpp \
	hppciProblem.cpp \
	hppciRobot.cpp \
	hppciServer.cpp \
	hppciServerPrivate.cpp

libhppCorbaServer_la_CPPFLAGS = \
	-I$(srcdir)/../include \
	-I. \
	-I@top_builddir@/idl/server


libhppCorbaServer_la_CPPFLAGS += \
	@HPPCORE_CFLAGS@ \
	@OMNIORB4_CFLAGS@ \
	@HPPOPENHRP_CFLAGS@ 


libhppCorbaServer_la_LIBADD = \
	@HPPCORE_LIBS@ \
	@OMNIORB4_LIBS@ \
	@HPPOPENHRP_LIBS@

%SK.cc: ../idl/server/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx $<

%SK.cc: @MODELLOADERJRL_IDL@/%.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx $<
