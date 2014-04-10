pluginsdir = $(prefix)/switcher-@LIBSWITCHER_API_VERSION@/plugins

plugins_LTLIBRARIES = \
    libpjsip_plugin.la 

#libpjsip info
libpjsip_plugin_la_SOURCES = \
    pj-sip.cpp \
    transport_adapter_sample.c

libpjsip_plugin_la_include_HEADERS = \
    pj-sip.h \
    transport_adapter_sample.h

libpjsip_plugin_la_includedir = \
    $(includedir)/switcher-$(LIBSWITCHER_API_VERSION)/switcher

libpjsip_plugin_la_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GST_LIBS) \
    $(GMODULE_LIBS) \
    $(JSONGLIB_LIBS) \
    $(PJSIP_LIBS) \
    $(SHMDATA_LIBS) \
    -avoid-version \
    -module \
    -L$(top_builddir)/switcher/.libs \
    -lswitcher-@LIBSWITCHER_API_VERSION@

libpjsip_plugin_la_CXXFLAGS = \
    $(GLIB_CFLAGS) \
    $(GMODULE_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(PJSIP_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -DG_LOG_DOMAIN=\"switcher\" \
    -I$(top_srcdir) \
    -Wno-error=write-strings


#### plugin unit test
AM_CFLAGS = \
    -I$(top_srcdir)

AM_CXXFLAGS =  \
    $(GLIB_CFLAGS) \
    $(GSOAP_CFLAGS) \
    $(GST_CFLAGS) \
    $(JSONGLIB_CFLAGS) \
    $(PJSIP_CFLAGS) \
    $(SHMDATA_CFLAGS) \
    -I$(top_srcdir) 

AM_LDFLAGS = \
    $(GLIB_LIBS) \
    $(GSOAP_LIBS) \
    $(GST_LIBS) \
    $(JSONGLIB_LIBS) \
    $(PJSIP_LIBS) \
    $(SHMDATA_LIBS) \
    $(top_builddir)/switcher/libswitcher-@LIBSWITCHER_API_VERSION@.la

check_PROGRAMS = \
    check_pjsip

check_pjsip_SOURCES = \
    check_pjsip.cpp

TESTS = $(check_PROGRAMS)