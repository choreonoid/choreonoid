#ifndef CNOID_CORBAPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_CORBAPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_CORBAPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_CORBAPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_CORBAPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_CORBAPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_CORBAPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_CORBAPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_CORBAPLUGIN_DLLIMPORT
#   define CNOID_CORBAPLUGIN_DLLEXPORT
#   define CNOID_CORBAPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_CORBAPLUGIN_STATIC
#  define CNOID_CORBAPLUGIN_DLLAPI
#  define CNOID_CORBAPLUGIN_LOCAL
# else
#  ifdef CnoidCorbaPlugin_EXPORTS
#   define CNOID_CORBAPLUGIN_DLLAPI CNOID_CORBAPLUGIN_DLLEXPORT
#  else
#   define CNOID_CORBAPLUGIN_DLLAPI CNOID_CORBAPLUGIN_DLLIMPORT
#  endif
#  define CNOID_CORBAPLUGIN_LOCAL CNOID_CORBAPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_CORBAPLUGIN_DLLAPI
