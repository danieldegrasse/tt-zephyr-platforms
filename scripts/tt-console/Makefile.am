AUTOMAKE_OPTIONS = subdir-objects
SUBDIRS=../../../modules/libjaylink
bin_PROGRAMS = tt-console
tt_console_SOURCES = console.c arc_tlb.c
tt_console_CPPFLAGS = -g -O0 -std=gnu11 -I ../../include -Wall -Werror
noinst_LTLIBRARIES = libjtag.la
libjtag_la_LIBADD = ../../../modules/libjaylink/libjaylink/libjaylink.la
libjtag_la_CPPFLAGS = -I ../../../modules/libjaylink/ -g -O0 -std=gnu11 -I ../../include -Wall -Werror
libjtag_la_SOURCES = arc_jtag.c

tt_console_LDADD = libjtag.la
