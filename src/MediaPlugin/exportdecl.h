  
#ifndef CNOID_MEDIAPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_MEDIAPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_MEDIAPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_MEDIAPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_MEDIAPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_MEDIAPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_MEDIAPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_MEDIAPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_MEDIAPLUGIN_DLLIMPORT
#   define CNOID_MEDIAPLUGIN_DLLEXPORT
#   define CNOID_MEDIAPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_MEDIAPLUGIN_STATIC
#  define CNOID_MEDIAPLUGIN_DLLAPI
#  define CNOID_MEDIAPLUGIN_LOCAL
# else
#  ifdef CnoidMediaPlugin_EXPORTS
#   define CNOID_MEDIAPLUGIN_DLLAPI CNOID_MEDIAPLUGIN_DLLEXPORT
#  else
#   define CNOID_MEDIAPLUGIN_DLLAPI CNOID_MEDIAPLUGIN_DLLIMPORT
#  endif
#  define CNOID_MEDIAPLUGIN_LOCAL CNOID_MEDIAPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_MEDIAPLUGIN_DLLAPI
