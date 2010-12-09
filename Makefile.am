include $(top_srcdir)/build-aux/init.mk

# Search for autoconf macros in build-aux.
ACLOCAL_AMFLAGS = -I build-aux

SUBDIRS = 		\
	idl		\
	include		\
	src		\
	doc

# Provide bootstrap with the generated package.
EXTRA_DIST +=	bootstrap			\
		build-aux/bootstrap


# Nothing to do in this directory.
all:

# Include various rules...
include $(top_srcdir)/build-aux/doc-recursive.mk
include $(top_srcdir)/build-aux/libtool.mk
include $(top_srcdir)/build-aux/pkg-config.mk
