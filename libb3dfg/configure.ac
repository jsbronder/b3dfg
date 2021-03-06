AC_INIT([libb3dfg], [1.0])
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([libb3dfg/core.c])
AM_CONFIG_HEADER([config.h])

AC_PREREQ([2.50])
AC_PROG_CC
AC_PROG_LIBTOOL
AC_C_INLINE
AM_PROG_CC_C_O

# Library versioning
lt_major="1"
lt_revision="0"
lt_age="0"
AC_SUBST(lt_major)
AC_SUBST(lt_revision)
AC_SUBST(lt_age)

# Examples build
AC_ARG_ENABLE([examples-build], [AS_HELP_STRING([--enable-examples-build],
	[build example applications (default n)])],
	[build_examples=$enableval],
	[build_examples='no'])
AM_CONDITIONAL([BUILD_EXAMPLES], [test "x$build_examples" != "xno"])

# Message logging
AC_ARG_ENABLE([log], [AS_HELP_STRING([--disable-log], [disable all logging])],
    [log_enabled=$enableval],
    [log_enabled='yes'])
if test "x$log_enabled" != "xno"; then
    AC_DEFINE([ENABLE_LOGGING], 1, [Message logging])
fi

AC_ARG_ENABLE([debug-log], [AS_HELP_STRING([--enable-debug-log],
    [enable debug logging (default n)])],
    [debug_log_enabled=$enableval],
    [debug_log_enabled='no'])
if test "x$debug_log_enabled" != "xno"; then
    AC_DEFINE([ENABLE_DEBUG_LOGGING], 1, [Debug message logging])
fi

# Restore gnu89 inline semantics on gcc 4.3 and newer
saved_cflags="$CFLAGS"
CFLAGS="$CFLAGS -fgnu89-inline"
AC_COMPILE_IFELSE(AC_LANG_PROGRAM([]), inline_cflags="-fgnu89-inline", inline_cflags="")
CFLAGS="$saved_cflags"

AC_DEFINE([API_EXPORTED], [__attribute__((visibility("default")))], [Default visibility])
AM_CFLAGS="-std=gnu99 $inline_cflags -Wall -Wundef -Wunused -Wstrict-prototypes -Werror-implicit-function-declaration -Wno-pointer-sign -Wshadow"
AC_SUBST(AM_CFLAGS)

AC_CONFIG_FILES([libb3dfg.pc] [Makefile] [libb3dfg/Makefile] [examples/Makefile] [doc/Makefile])
AC_OUTPUT

