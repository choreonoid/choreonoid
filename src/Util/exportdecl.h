#ifndef CNOID_UTIL_EXPORTDECL_H_INCLUDED
# define CNOID_UTIL_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_UTIL_DLLIMPORT __declspec(dllimport)
#  define CNOID_UTIL_DLLEXPORT __declspec(dllexport)
#  define CNOID_UTIL_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_UTIL_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_UTIL_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_UTIL_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_UTIL_DLLIMPORT
#   define CNOID_UTIL_DLLEXPORT
#   define CNOID_UTIL_DLLLOCAL
#  endif
# endif

# ifdef CNOID_UTIL_STATIC
#  define CNOID_UTIL_DLLAPI
#  define CNOID_UTIL_LOCAL
# else
#  ifdef CnoidUtil_EXPORTS
#   define CNOID_UTIL_DLLAPI CNOID_UTIL_DLLEXPORT
#  else
#   define CNOID_UTIL_DLLAPI CNOID_UTIL_DLLIMPORT
#  endif
#  define CNOID_UTIL_LOCAL CNOID_UTIL_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_UTIL_DLLAPI
