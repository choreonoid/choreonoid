#ifndef CNOID_LUA_BASE_EXPORTDECL_H_INCLUDED
# define CNOID_LUA_BASE_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_LUA_BASE_DLLIMPORT __declspec(dllimport)
#  define CNOID_LUA_BASE_DLLEXPORT __declspec(dllexport)
#  define CNOID_LUA_BASE_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_LUA_BASE_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_LUA_BASE_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_LUA_BASE_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_LUA_BASE_DLLIMPORT
#   define CNOID_LUA_BASE_DLLEXPORT
#   define CNOID_LUA_BASE_DLLLOCAL
#  endif
# endif

# ifdef CNOID_LUA_BASE_STATIC
#  define CNOID_LUA_BASE_DLLAPI
#  define CNOID_LUA_BASE_LOCAL
# else
#  ifdef CnoidLuaBase_EXPORTS
#   define CNOID_LUA_BASE_DLLAPI CNOID_LUA_BASE_DLLEXPORT
#  else
#   define CNOID_LUA_BASE_DLLAPI CNOID_LUA_BASE_DLLIMPORT
#  endif
#  define CNOID_LUA_BASE_LOCAL CNOID_LUA_BASE_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_LUA_BASE_DLLAPI
