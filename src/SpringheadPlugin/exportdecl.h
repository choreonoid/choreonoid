#ifndef CNOID_SPRINGHEADPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_SPRINGHEADPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_SPRINGHEADPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_SPRINGHEADPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_SPRINGHEADPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_SPRINGHEADPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_SPRINGHEADPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_SPRINGHEADPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_SPRINGHEADPLUGIN_DLLIMPORT
#   define CNOID_SPRINGHEADPLUGIN_DLLEXPORT
#   define CNOID_SPRINGHEADPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_SPRINGHEADPLUGIN_STATIC
#  define CNOID_SPRINGHEADPLUGIN_DLLAPI
#  define CNOID_SPRINGHEADPLUGIN_LOCAL
# else
#  ifdef CnoidSpringheadPlugin_EXPORTS
#   define CNOID_SPRINGHEADPLUGIN_DLLAPI CNOID_SPRINGHEADPLUGIN_DLLEXPORT
#  else
#   define CNOID_SPRINGHEADPLUGIN_DLLAPI CNOID_SPRINGHEADPLUGIN_DLLIMPORT
#  endif
#  define CNOID_SPRINGHEADPLUGIN_LOCAL CNOID_SPRINGHEADPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_SPRINGHEADPLUGIN_DLLAPI
