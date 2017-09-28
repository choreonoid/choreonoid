  
#ifndef CNOID_LUAPLUGIN_EXPORTDECL_H_INCLUDED
# define CNOID_LUAPLUGIN_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_LUAPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_LUAPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_LUAPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_LUAPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_LUAPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_LUAPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_LUAPLUGIN_DLLIMPORT
#   define CNOID_LUAPLUGIN_DLLEXPORT
#   define CNOID_LUAPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_LUAPLUGIN_STATIC
#  define CNOID_LUAPLUGIN_DLLAPI
#  define CNOID_LUAPLUGIN_LOCAL
# else
#  ifdef CnoidLuaPlugin_EXPORTS
#   define CNOID_LUAPLUGIN_DLLAPI CNOID_LUAPLUGIN_DLLEXPORT
#  else
#   define CNOID_LUAPLUGIN_DLLAPI CNOID_LUAPLUGIN_DLLIMPORT
#  endif
#  define CNOID_LUAPLUGIN_LOCAL CNOID_LUAPLUGIN_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_LUAPLUGIN_DLLAPI
