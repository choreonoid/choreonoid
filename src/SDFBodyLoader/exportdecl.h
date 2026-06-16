#ifndef CNOID_SDF_BODY_LOADER_EXPORTDECL_H
# define CNOID_SDF_BODY_LOADER_EXPORTDECL_H

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_SDF_BODY_LOADER_DLLIMPORT __declspec(dllimport)
#  define CNOID_SDF_BODY_LOADER_DLLEXPORT __declspec(dllexport)
#  define CNOID_SDF_BODY_LOADER_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_SDF_BODY_LOADER_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_SDF_BODY_LOADER_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_SDF_BODY_LOADER_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_SDF_BODY_LOADER_DLLIMPORT
#   define CNOID_SDF_BODY_LOADER_DLLEXPORT
#   define CNOID_SDF_BODY_LOADER_DLLLOCAL
#  endif
# endif

# ifdef CNOID_SDF_BODY_LOADER_STATIC
#  define CNOID_SDF_BODY_LOADER_DLLAPI
#  define CNOID_SDF_BODY_LOADER_LOCAL
# else
#  ifdef CnoidSDFBodyLoader_EXPORTS
#   define CNOID_SDF_BODY_LOADER_DLLAPI CNOID_SDF_BODY_LOADER_DLLEXPORT
#  else
#   define CNOID_SDF_BODY_LOADER_DLLAPI CNOID_SDF_BODY_LOADER_DLLIMPORT
#  endif
#  define CNOID_SDF_BODY_LOADER_LOCAL CNOID_SDF_BODY_LOADER_DLLLOCAL
# endif

#endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_SDF_BODY_LOADER_DLLAPI
