# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

hppCorbaServerdir = $(datadir)/doc/hppCorbaServer/html

EXTRA_DIST = \
	header.html \
	footer.html \
	hpp.css \
	additionalDoc/hppCore.h \
	pictures/HRP2.jpg


install: html-doc
	mkdir -p $(hppCorbaServerdir)
	$(INSTALL_DATA) html/* $(hppCorbaServerdir)
	mkdir -p $(hppCorbaServerdir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(hppCorbaServerdir)/pictures
	$(INSTALL_DATA) hppCorbaServer.doxytag $(hppCorbaServerdir)

html-doc: hpp.dox
	doxygen $<
