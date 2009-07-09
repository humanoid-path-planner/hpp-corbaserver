include $(top_srcdir)/build-aux/init.mk

# --- Distribute and install headers.
nobase_include_HEADERS =			 	\
	hppCorbaServer/hppciServer.h			\
	hppCorbaServer/hppciObstacle.h			\
	hppCorbaServer/hppciProblem.h			\
	hppCorbaServer/hppciRobot.h

if OPENHRP
 nobase_include_HEADERS +=			 	\
	hppCorbaServer/hppciOpenHrp.h
endif

EXTRA_DIST += 						\
	hppCorbaServer/hppciTools.h 			\
	hppCorbaServer/hppciServerPrivate.h
