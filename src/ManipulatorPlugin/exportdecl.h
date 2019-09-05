#ifndef CNOID_MANIPULATOR_EXPORTDECL_H
# define CNOID_MANIPULATOR_EXPORTDECL_H
# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_MANIPULATOR_DLLIMPORT __declspec(dllimport)
#  define CNOID_MANIPULATOR_DLLEXPORT __declspec(dllexport)
#  define CNOID_MANIPULATOR_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_MANIPULATOR_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_MANIPULATOR_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_MANIPULATOR_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_MANIPULATOR_DLLIMPORT
#   define CNOID_MANIPULATOR_DLLEXPORT
#   define CNOID_MANIPULATOR_DLLLOCAL
#  endif
# endif

# ifdef CNOID_MANIPULATOR_STATIC
#  define CNOID_MANIPULATOR_DLLAPI
#  define CNOID_MANIPULATOR_LOCAL
# else
#  ifdef CnoidManipulator_EXPORTS
#   define CNOID_MANIPULATOR_DLLAPI CNOID_MANIPULATOR_DLLEXPORT
#  else
#   define CNOID_MANIPULATOR_DLLAPI CNOID_MANIPULATOR_DLLIMPORT
#  endif
#  define CNOID_MANIPULATOR_LOCAL CNOID_MANIPULATOR_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_MANIPULATOR_DLLAPI
