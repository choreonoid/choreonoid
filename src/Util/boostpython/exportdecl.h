#ifndef CNOID_PYUTIL_EXPORTDECL_H
# define CNOID_PYUTIL_EXPORTDECL_H

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_PYUTIL_DLLIMPORT __declspec(dllimport)
#  define CNOID_PYUTIL_DLLEXPORT __declspec(dllexport)
#  define CNOID_PYUTIL_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_PYUTIL_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_PYUTIL_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_PYUTIL_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_PYUTIL_DLLIMPORT
#   define CNOID_PYUTIL_DLLEXPORT
#   define CNOID_PYUTIL_DLLLOCAL
#  endif
# endif

# ifdef CNOID_PYUTIL_STATIC
#  define CNOID_PYUTIL_DLLAPI
#  define CNOID_PYUTIL_LOCAL
# else
#  ifdef CnoidPyUtil_EXPORTS
#   define CNOID_PYUTIL_DLLAPI CNOID_PYUTIL_DLLEXPORT
#  else
#   define CNOID_PYUTIL_DLLAPI CNOID_PYUTIL_DLLIMPORT
#  endif
#  define CNOID_PYUTIL_LOCAL CNOID_PYUTIL_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_PYUTIL_DLLAPI
