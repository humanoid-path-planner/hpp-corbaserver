# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

SUBDIRS = \
	idl/server \
	src \
	include

hppdir = $(libdir)/pkgconfig
hpp_DATA = hppCorbaServer.pc

install-data-hook: documentation

documentation:
	cd doc; $(MAKE) install
