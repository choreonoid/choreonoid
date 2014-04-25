#ifndef CNOID_PCLPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_PCLPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_PCLPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_PCLPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_PCLPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_PCLPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_PCLPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_PCLPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_PCLPLUGIN_DLLIMPORT
#   define CNOID_PCLPLUGIN_DLLEXPORT
#   define CNOID_PCLPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_PCLPLUGIN_STATIC
#  define CNOID_PCLPLUGIN_DLLAPI
#  define CNOID_PCLPLUGIN_LOCAL
# else
#  ifdef CnoidPCLPlugin_EXPORTS
#   define CNOID_PCLPLUGIN_DLLAPI CNOID_PCLPLUGIN_DLLEXPORT
#  else
#   define CNOID_PCLPLUGIN_DLLAPI CNOID_PCLPLUGIN_DLLIMPORT
#  endif
#  define CNOID_PCLPLUGIN_LOCAL CNOID_PCLPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_PCLPLUGIN_DLLAPI
