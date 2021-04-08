#ifndef CNOID_URDFPLUGIN_EXPORTDECL_H
# define CNOID_URDFPLUGIN_EXPORTDECL_H

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_URDFPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_URDFPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_URDFPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_URDFPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_URDFPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_URDFPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_URDFPLUGIN_DLLIMPORT
#   define CNOID_URDFPLUGIN_DLLEXPORT
#   define CNOID_URDFPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_URDFPLUGIN_STATIC
#  define CNOID_URDFPLUGIN_DLLAPI
#  define CNOID_URDFPLUGIN_LOCAL
# else
#  ifdef CnoidMediaPlugin_EXPORTS
#   define CNOID_URDFPLUGIN_DLLAPI CNOID_URDFPLUGIN_DLLEXPORT
#  else
#   define CNOID_URDFPLUGIN_DLLAPI CNOID_URDFPLUGIN_DLLIMPORT
#  endif
#  define CNOID_URDFPLUGIN_LOCAL CNOID_URDFPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_URDFPLUGIN_DLLAPI
