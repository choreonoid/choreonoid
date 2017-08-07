#ifndef CNOID_PYTHON3_EXPORTDECL_H_INCLUDED
# define CNOID_PYTHON3_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_PYTHON3_DLLIMPORT __declspec(dllimport)
#  define CNOID_PYTHON3_DLLEXPORT __declspec(dllexport)
#  define CNOID_PYTHON3_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_PYTHON3_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_PYTHON3_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_PYTHON3_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_PYTHON3_DLLIMPORT
#   define CNOID_PYTHON3_DLLEXPORT
#   define CNOID_PYTHON3_DLLLOCAL
#  endif
# endif

# ifdef CNOID_PYTHON3_STATIC
#  define CNOID_PYTHON3_DLLAPI
#  define CNOID_PYTHON3_LOCAL
# else
#  ifdef CnoidPython3_EXPORTS
#   define CNOID_PYTHON3_DLLAPI CNOID_PYTHON3_DLLEXPORT
#  else
#   define CNOID_PYTHON3_DLLAPI CNOID_PYTHON3_DLLIMPORT
#  endif
#  define CNOID_PYTHON3_LOCAL CNOID_PYTHON3_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_PYTHON3_DLLAPI
