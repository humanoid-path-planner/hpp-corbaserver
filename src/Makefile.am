include $(top_srcdir)/build-aux/init.mk
include $(top_srcdir)/build-aux/kineo.mk

include $(top_srcdir)/shared.mk

BUILT_SOURCES=

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

# Corba server library.
lib_LTLIBRARIES = libhpp-corbaserver.la

libhpp_corbaserver_la_SOURCES =		\
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

libhpp_corbaserver_la_LIBADD = 					\
	$(top_builddir)/idl/hpp/corbaserver/libcorba-stubs.la	\
	$(ROBOTBUILDER_LIBS)					\
	$(HPPCORE_LIBS) 					\
	$(HPP_UTIL_LIBS)					\
	$(KWSPLUS_LIBS)						\
	$(HRP2DYNAMICS_LIBS)

if OPENHRP

IDL_PREFIX= $(top_builddir)/src
IDL_DIRECTORY= $(IDL_PREFIX)
IDL_HEADER_STUB_SUFFIX= .hh
IDL_GUARD_PREFIX = "HPP_CORBASERVER_"
include $(top_srcdir)/build-aux/idl.mk

#FIXME: search for moderlloader IDLs using MODELLOADERIDL variable.
#...and generates stubs from it.
common-modelloader.idl:
	cp $(MODELLOADERJRL_IDL)/$@ .
jrl-modelloader.idl:
	cp $(MODELLOADERJRL_IDL)/$@ .

common-modelloader.stub.cc common-modelloader.hh:	\
		common-modelloader.idl			\
		$(top_builddir)/config.status
	$(OMNIIDL) $(OMNIIDLFLAGS) "$<"

jrl-modelloader.stub.cc jrl-modelloader.hh:	\
		jrl-modelloader.idl		\
		$(top_builddir)/config.status
	$(OMNIIDL) $(OMNIIDLFLAGS) "$<"

BUILT_SOURCES +=				\
		common-modelloader.idl		\
		jrl-modelloader.idl		\
		common-modelloader.stub.cc	\
		common-modelloader.hh 		\
		jrl-modelloader.stub.cc		\
		jrl-modelloader.hh

nodist_libhpp_corbaserver_la_SOURCES =		\
	common-modelloader.stub.cc		\
	common-modelloader.hh			\
	jrl-modelloader.stub.cc			\
	jrl-modelloader.hh


libhpp_corbaserver_la_SOURCES +=		\
	openhrp.cc

libhpp_corbaserver_la_LIBADD += 		\
	$(HPPOPENHRP_LIBS) $(ROBOTBUILDER_LIBS)

endif



# Corba server executable.
bin_PROGRAMS = hpp-corbaserver
hpp_corbaserver_SOURCES = hpp-corbaserver.cc
hpp_corbaserver_LDADD = libhpp-corbaserver.la
BUILT_SOURCES += hpp-corbaserver.kab
