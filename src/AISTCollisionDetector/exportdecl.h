#ifndef CNOID_COLLISION_EXPORTDECL_H_INCLUDED
# define CNOID_COLLISION_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_COLLISION_DLLIMPORT __declspec(dllimport)
#  define CNOID_COLLISION_DLLEXPORT __declspec(dllexport)
#  define CNOID_COLLISION_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_COLLISION_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_COLLISION_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_COLLISION_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_COLLISION_DLLIMPORT
#   define CNOID_COLLISION_DLLEXPORT
#   define CNOID_COLLISION_DLLLOCAL
#  endif
# endif

# ifdef CNOID_COLLISION_STATIC
#  define CNOID_COLLISION_DLLAPI
#  define CNOID_COLLISION_LOCAL
# else
#  ifdef CnoidAISTCollisionDetector_EXPORTS
#   define CNOID_COLLISION_DLLAPI CNOID_COLLISION_DLLEXPORT
#  else
#   define CNOID_COLLISION_DLLAPI CNOID_COLLISION_DLLIMPORT
#  endif
#  define CNOID_COLLISION_LOCAL CNOID_COLLISION_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_COLLISION_DLLAPI

