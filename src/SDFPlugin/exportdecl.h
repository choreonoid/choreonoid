#ifndef CNOID_SDFPLUGIN_EXPORTDECL_H
# define CNOID_SDFPLUGIN_EXPORTDECL_H

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_SDFPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_SDFPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_SDFPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_SDFPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_SDFPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_SDFPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_SDFPLUGIN_DLLIMPORT
#   define CNOID_SDFPLUGIN_DLLEXPORT
#   define CNOID_SDFPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_SDFPLUGIN_STATIC
#  define CNOID_SDFPLUGIN_DLLAPI
#  define CNOID_SDFPLUGIN_LOCAL
# else
#  ifdef CnoidSDFPlugin_EXPORTS
#   define CNOID_SDFPLUGIN_DLLAPI CNOID_SDFPLUGIN_DLLEXPORT
#  else
#   define CNOID_SDFPLUGIN_DLLAPI CNOID_SDFPLUGIN_DLLIMPORT
#  endif
#  define CNOID_SDFPLUGIN_LOCAL CNOID_SDFPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_SDFPLUGIN_DLLAPI
