#ifndef CNOID_BODYPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_BODYPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_BODYPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_BODYPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_BODYPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_BODYPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_BODYPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_BODYPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_BODYPLUGIN_DLLIMPORT
#   define CNOID_BODYPLUGIN_DLLEXPORT
#   define CNOID_BODYPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_BODYPLUGIN_STATIC
#  define CNOID_BODYPLUGIN_DLLAPI
#  define CNOID_BODYPLUGIN_LOCAL
# else
#  ifdef CnoidBodyPlugin_EXPORTS
#   define CNOID_BODYPLUGIN_DLLAPI CNOID_BODYPLUGIN_DLLEXPORT
#  else
#   define CNOID_BODYPLUGIN_DLLAPI CNOID_BODYPLUGIN_DLLIMPORT
#  endif
#  define CNOID_BODYPLUGIN_LOCAL CNOID_BODYPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_BODYPLUGIN_DLLAPI
