# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

hppCorbaServerdir = @includedir@/hpp
hppCorbaServer_HEADERS = \
	hppciServer.h		\
	hppciObstacle.h		\
	hppciProblem.h		\
	hppciRobot.h

if OPENHRP
install-data-hook: hppciOpenHrp

hppciOpenHrp:
	$(INSTALL_DATA) hppciOpenHrp.h @includedir@/hpp
endif

EXTRA_DIST=\
	hppciServerPrivate.h

