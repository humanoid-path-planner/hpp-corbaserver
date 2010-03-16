include $(top_srcdir)/build-aux/init.mk

IDL_DIRECTORY= $(IDL_PREFIX)/hpp/corbaserver
IDL_GUARD_PREFIX = "HPP_CORBASERVER_"
include $(top_srcdir)/build-aux/idl.mk

include $(top_srcdir)/shared.mk

# Compile stubs separately to save recompilation time.
# Do *not* include C++ warnings as generated code triggers
# some (-Werror would then make the compilation fail).
noinst_LTLIBRARIES = libcorba-stubs.la
nodist_libcorba_stubs_la_SOURCES =	\
	common.stub.cc			\
	common.stub.hh			\
	obstacle.stub.cc		\
	obstacle.stub.hh		\
	problem.stub.cc			\
	problem.stub.hh			\
	robot.stub.cc			\
	robot.stub.hh
