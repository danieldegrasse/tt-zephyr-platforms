AUTOMAKE_OPTIONS = subdir-objects
SUBDIRS=../../../modules/libjaylink
bin_PROGRAMS = tt-console
tt_console_SOURCES = console.c
tt_console_CPPFLAGS = -g -O0 -std=gnu11 -I ../../include
noinst_LTLIBRARIES = libjtag.la
libjtag_la_LIBADD = ../../../modules/libjaylink/libjaylink/libjaylink.la
libjtag_la_CPPFLAGS = -I ../../../modules/libjaylink/
libjtag_la_SOURCES = jlink.c

tt_console_LDADD = libjtag.la
