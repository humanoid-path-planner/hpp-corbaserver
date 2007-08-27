# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

hppCoredir = $(datadir)/doc/hppCorbaServer

EXTRA_DIST = \
	header.html \
	footer.html \
	hpp.css \
	additionalDoc/hppCore.h \
	pictures/HRP2.jpg


install: html-doc
	mkdir -p $(hppCoredir)
	$(INSTALL_DATA) html/* $(hppCoredir)
	mkdir -p $(hppCoredir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(hppCoredir)/pictures
	mkdir -p $(datadir)/doc/doxytag
	$(INSTALL_DATA) hppCorbaServer.doxytag $(datadir)/doc/doxytag

html-doc: hpp.dox
	doxygen $<
