  
#ifndef CNOID_GRXUIPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_GRXUIPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_GRXUIPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_GRXUIPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_GRXUIPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_GRXUIPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_GRXUIPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_GRXUIPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_GRXUIPLUGIN_DLLIMPORT
#   define CNOID_GRXUIPLUGIN_DLLEXPORT
#   define CNOID_GRXUIPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_GRXUIPLUGIN_STATIC
#  define CNOID_GRXUIPLUGIN_DLLAPI
#  define CNOID_GRXUIPLUGIN_LOCAL
# else
#  ifdef CnoidGrxUIPlugin_EXPORTS
#   define CNOID_GRXUIPLUGIN_DLLAPI CNOID_GRXUIPLUGIN_DLLEXPORT
#  else
#   define CNOID_GRXUIPLUGIN_DLLAPI CNOID_GRXUIPLUGIN_DLLIMPORT
#  endif
#  define CNOID_GRXUIPLUGIN_LOCAL CNOID_GRXUIPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_GRXUIPLUGIN_DLLAPI
