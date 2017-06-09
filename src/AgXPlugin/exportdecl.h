#ifndef CNOID_AGXPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_AGXPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_AGXPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_AGXPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_AGXPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_AGXPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_AGXPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_AGXPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_AGXPLUGIN_DLLIMPORT
#   define CNOID_AGXPLUGIN_DLLEXPORT
#   define CNOID_AGXPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_AGXPLUGIN_STATIC
#  define CNOID_AGXPLUGIN_DLLAPI
#  define CNOID_AGXPLUGIN_LOCAL
# else
#  ifdef CnoidAgXPlugin_EXPORTS
#   define CNOID_AGXPLUGIN_DLLAPI CNOID_AGXPLUGIN_DLLEXPORT
#  else
#   define CNOID_AGXPLUGIN_DLLAPI CNOID_AGXPLUGIN_DLLIMPORT
#  endif
#  define CNOID_AGXPLUGIN_LOCAL CNOID_AGXPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_AGXPLUGIN_DLLAPI
