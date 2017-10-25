#ifndef CNOID_BODYIORTC_EXPORTDECL_H_INCLUDED
# define CNOID_BODYIORTC_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_BODYIORTC_DLLIMPORT __declspec(dllimport)
#  define CNOID_BODYIORTC_DLLEXPORT __declspec(dllexport)
#  define CNOID_BODYIORTC_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_BODYIORTC_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_BODYIORTC_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_BODYIORTC_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_BODYIORTC_DLLIMPORT
#   define CNOID_BODYIORTC_DLLEXPORT
#   define CNOID_BODYIORTC_DLLLOCAL
#  endif
# endif

# ifdef CNOID_BODYIORTC_STATIC
#  define CNOID_BODYIORTC_DLLAPI
#  define CNOID_BODYIORTC_LOCAL
# else
#  ifdef CnoidBodyIoRTC_EXPORTS
#   define CNOID_BODYIORTC_DLLAPI CNOID_BODYIORTC_DLLEXPORT
#  else
#   define CNOID_BODYIORTC_DLLAPI CNOID_BODYIORTC_DLLIMPORT
#  endif
#  define CNOID_BODYIORTC_LOCAL CNOID_BODYIORTC_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_BODYIORTC_DLLAPI
