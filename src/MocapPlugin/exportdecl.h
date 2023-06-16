#ifndef CNOID_MOCAPPLUGIN_EXPORTDECL_H
# define CNOID_MOCAPPLUGIN_EXPORTDECL_H

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_MOCAPPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_MOCAPPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_MOCAPPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_MOCAPPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_MOCAPPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_MOCAPPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_MOCAPPLUGIN_DLLIMPORT
#   define CNOID_MOCAPPLUGIN_DLLEXPORT
#   define CNOID_MOCAPPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_MOCAPPLUGIN_STATIC
#  define CNOID_MOCAPPLUGIN_DLLAPI
#  define CNOID_MOCAPPLUGIN_LOCAL
# else
#  ifdef CnoidMocapPlugin_EXPORTS
#   define CNOID_MOCAPPLUGIN_DLLAPI CNOID_MOCAPPLUGIN_DLLEXPORT
#  else
#   define CNOID_MOCAPPLUGIN_DLLAPI CNOID_MOCAPPLUGIN_DLLIMPORT
#  endif
#  define CNOID_MOCAPPLUGIN_LOCAL CNOID_MOCAPPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_MOCAPPLUGIN_DLLAPI
