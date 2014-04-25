#ifndef CNOID_CORBA_EXPORTDECL_H_INCLUDED
# define CNOID_CORBA_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_CORBA_DLLIMPORT __declspec(dllimport)
#  define CNOID_CORBA_DLLEXPORT __declspec(dllexport)
#  define CNOID_CORBA_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_CORBA_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_CORBA_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_CORBA_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_CORBA_DLLIMPORT
#   define CNOID_CORBA_DLLEXPORT
#   define CNOID_CORBA_DLLLOCAL
#  endif
# endif

# ifdef CNOID_CORBA_STATIC
#  define CNOID_CORBA_DLLAPI
#  define CNOID_CORBA_LOCAL
# else
#  ifdef CnoidCorba_EXPORTS
#   define CNOID_CORBA_DLLAPI CNOID_CORBA_DLLEXPORT
#  else
#   define CNOID_CORBA_DLLAPI CNOID_CORBA_DLLIMPORT
#  endif
#  define CNOID_CORBA_LOCAL CNOID_CORBA_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_CORBA_DLLAPI
