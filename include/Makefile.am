# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

hppCorbaServerdir = @includedir@/hppCorbaServer
hppCorbaServer_HEADERS = \
	hppCorbaServer/hppciServer.h		\
	hppCorbaServer/hppciObstacle.h		\
	hppCorbaServer/hppciProblem.h		\
	hppCorbaServer/hppciRobot.h

if OPENHRP
install-data-hook: hppciOpenHrp

hppciOpenHrp:
	$(INSTALL_DATA) hppCorbaServer/hppciOpenHrp.h @includedir@/hppCorbaServer
endif

EXTRA_DIST = \
	hppCorbaServer/hppciTools.h \
	hppCorbaServer/hppciServerPrivate.h 
