include $(top_srcdir)/build-aux/init.mk
include $(top_srcdir)/build-aux/kineo.mk

# Corba server library.
lib_LTLIBRARIES = libhpp-corbaserver.la

nodist_libhpp_corbaserver_la_SOURCES =	\
	obstacle.impl.cc 		\
	obstacle.impl.hh 		\
	problem.impl.cc 		\
	problem.impl.hh 		\
	robot.impl.cc 			\
	robot.impl.hh 			\
	server.cc 			\
	server-private.cc 		\
	server-private.hh		\
	tools.cc			\
	tools.hh

libhpp_corbaserver_la_LIBADD = 		\
	$(top_builddir)/idl/hpp/corbaserver/libcorba-stubs.la

if OPENHRP

nodist_libhpp_corbaserver_la_SOURCES +=	\
	common-modelloader.hh		\
	jrl-modelloader.hh

endif



# Corba server executable.
bin_PROGRAMS = hpp-corbaserver
hpp_corbaserver_SOURCES = hpp-corbaserver.cc
hpp_corbaserver_LDADD = libhpp-corbaserver.la
BUILT_SOURCES += hpp-corbaserver.kab
