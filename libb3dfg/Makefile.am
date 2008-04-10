AUTOMAKE_OPTIONS = foreign

EXTRA_DIST = libb3dfg.pc.in
DISTCLEANFILES = ChangeLog libb3dfg.pc

SUBDIRS = libb3dfg doc

if BUILD_EXAMPLES
SUBDIRS += examples
endif

pkgconfigdir=$(libdir)/pkgconfig
pkgconfig_DATA=libb3dfg.pc

.PHONY: ChangeLog
ChangeLog:
	git --git-dir $(top_srcdir)/.git log libb3dfg > ChangeLog || touch ChangeLog

dist-hook: ChangeLog

