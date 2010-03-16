# Add shared dependencies.
AM_CPPFLAGS +=			\
	-I$(top_builddir)/idl	\
	$(HPPCORE_CFLAGS)	\
	$(OMNIORB4_CFLAGS)

AM_LDFLAGS +=			\
	$(HPPCORE_LDFLAGS)	\
	$(OMNIORB4_LDFLAGS)

# Additional dependencies if OpenHRP support is enabled.
if OPENHRP

AM_CPPFLAGS +=			\
	$(HRP2DYNAMICS_CFLAGS)	\
	$(HPPOPENHRP_CFLAGS)

AM_LDFLAGS +=			\
	$(HRP2DYNAMICS_LIBS)	\
	$(HPPOPENHRP_LIBS)

endif
