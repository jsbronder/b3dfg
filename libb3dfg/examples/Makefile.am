INCLUDES = -I$(top_srcdir)
noinst_PROGRAMS = capture_frames poll test test_intr
noinst_LTLIBRARIES = libserial_cmd.la
extra_DIST = serial_cmd.h

libserial_cmd_la_SOURCES = serial_cmd.c

capture_frames_SOURCES = capture_frames.c
capture_frames_LDADD = ../libb3dfg/libb3dfg.la libserial_cmd.la -lb3dfg 

poll_SOURCES = poll.c
poll_LDADD = ../libb3dfg/libb3dfg.la libserial_cmd.la -lb3dfg

test_SOURCES = test.c
test_LDADD = ../libb3dfg/libb3dfg.la -lb3dfg libserial_cmd.la -lrt

test_intr_SOURCES = test_intr.c
test_intr_LDADD = ../libb3dfg/libb3dfg.la libserial_cmd.la -lb3dfg -lrt
