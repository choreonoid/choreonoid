#ifndef CNOID_BODY_EXPORTDECL_H
# define CNOID_BODY_EXPORTDECL_H
# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_BODY_DLLIMPORT __declspec(dllimport)
#  define CNOID_BODY_DLLEXPORT __declspec(dllexport)
#  define CNOID_BODY_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_BODY_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_BODY_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_BODY_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_BODY_DLLIMPORT
#   define CNOID_BODY_DLLEXPORT
#   define CNOID_BODY_DLLLOCAL
#  endif
# endif

# ifdef CNOID_BODY_STATIC
#  define CNOID_BODY_DLLAPI
#  define CNOID_BODY_LOCAL
# else
#  ifdef CnoidBody_EXPORTS
#   define CNOID_BODY_DLLAPI CNOID_BODY_DLLEXPORT
#  else
#   define CNOID_BODY_DLLAPI CNOID_BODY_DLLIMPORT
#  endif
#  define CNOID_BODY_LOCAL CNOID_BODY_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_BODY_DLLAPI
