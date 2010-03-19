include $(top_srcdir)/build-aux/init.mk
include $(top_srcdir)/build-aux/autotest.mk
include $(top_srcdir)/build-aux/kineo.mk

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

# Package the following files.
EXTRA_DIST += package.m4.in

# Clean atconfig (do not distribute it).
CLEANFILES += atconfig

# Source which are shared by all tests.
COMMON_SOURCES = common.hh

BUILT_SOURCES =

# simple-test
check_PROGRAMS = simple-test
simple_test_SOURCES = simple-test.cc $(COMMON_SOURCES)
simple_test_LDADD = $(top_builddir)/src/libhpp-corbaserver.la


.PHONY: generate-reference

# Re-generate reference outputs.
generate-reference: $(check_PROGRAMS)
	@echo "Generating references...";	\
	for i in $(check_PROGRAMS); do		\
	 IN="$(srcdir)/$$i.stdin";		\
	 OUT="$(srcdir)/$$i.stdout";		\
	 if test -f "$$IN"; then 		\
	  "./$$i" < $$IN > $$OUT || true;	\
	 else					\
	  "./$$i" > $$OUT || true;		\
	 fi					\
	done


# ------------ #
# Test suite.  #
# ------------ #

# Generate Kineo signatures.
BUILT_SOURCES += simple-test.kab
CLEANFILES += $(BUILT_SOURCES)

# Distribute reference files.
EXTRA_DIST +=					\
	simple-test.stdout
