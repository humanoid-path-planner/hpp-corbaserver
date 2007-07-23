# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

SUFFIXES = .idl

IDL_SRCS = \
	hppciProblemServer.idl \
	hppciRobotServer.idl \
	hppciObstacleServer.idl \
	hppciCommonServer.idl  

CORBA_STUBS_CC = $(IDL_SRCS:%.idl=%SK.cc)
CORBA_STUBS_HH = $(IDL_SRCS:%.idl=%.hh)

idldir = @datadir@/idl/hppCorbaServer
idl_DATA = $(IDL_SRCS)

corba_stubs: $(CORBA_STUBS_CC) $(CORBA_STUBS_HH)

%SK.cc: %.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx $<

%.hh: %.idl
	@OMNIORB_PREFIX@/bin/omniidl -bcxx $<

