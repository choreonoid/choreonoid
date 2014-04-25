#ifndef CNOID_ODEPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_ODEPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_ODEPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_ODEPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_ODEPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_ODEPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_ODEPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_ODEPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_ODEPLUGIN_DLLIMPORT
#   define CNOID_ODEPLUGIN_DLLEXPORT
#   define CNOID_ODEPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_ODEPLUGIN_STATIC
#  define CNOID_ODEPLUGIN_DLLAPI
#  define CNOID_ODEPLUGIN_LOCAL
# else
#  ifdef CnoidODEPlugin_EXPORTS
#   define CNOID_ODEPLUGIN_DLLAPI CNOID_ODEPLUGIN_DLLEXPORT
#  else
#   define CNOID_ODEPLUGIN_DLLAPI CNOID_ODEPLUGIN_DLLIMPORT
#  endif
#  define CNOID_ODEPLUGIN_LOCAL CNOID_ODEPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_ODEPLUGIN_DLLAPI
