#ifndef CNOID_PYTHON_EXPORTDECL_H_INCLUDED
# define CNOID_PYTHON_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_PYTHON_DLLIMPORT __declspec(dllimport)
#  define CNOID_PYTHON_DLLEXPORT __declspec(dllexport)
#  define CNOID_PYTHON_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_PYTHON_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_PYTHON_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_PYTHON_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_PYTHON_DLLIMPORT
#   define CNOID_PYTHON_DLLEXPORT
#   define CNOID_PYTHON_DLLLOCAL
#  endif
# endif

# ifdef CNOID_PYTHON_STATIC
#  define CNOID_PYTHON_DLLAPI
#  define CNOID_PYTHON_LOCAL
# else
#  ifdef CnoidPython_EXPORTS
#   define CNOID_PYTHON_DLLAPI CNOID_PYTHON_DLLEXPORT
#  else
#   define CNOID_PYTHON_DLLAPI CNOID_PYTHON_DLLIMPORT
#  endif
#  define CNOID_PYTHON_LOCAL CNOID_PYTHON_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_PYTHON_DLLAPI
