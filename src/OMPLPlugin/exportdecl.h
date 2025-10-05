#ifndef CNOID_OMPL_PLUGIN_EXPORTDECL_H
# define CNOID_OMPL_PLUGIN_EXPORTDECL_H

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define CNOID_OMPL_DLLIMPORT __declspec(dllimport)
#  define CNOID_OMPL_DLLEXPORT __declspec(dllexport)
#  define CNOID_OMPL_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define CNOID_OMPL_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_OMPL_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_OMPL_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define CNOID_OMPL_DLLIMPORT
#   define CNOID_OMPL_DLLEXPORT
#   define CNOID_OMPL_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef CNOID_OMPL_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define CNOID_OMPL_DLLAPI
#  define CNOID_OMPL_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef CnoidOMPLPlugin_EXPORTS
#   define CNOID_OMPL_DLLAPI CNOID_OMPL_DLLEXPORT
#  else
#   define CNOID_OMPL_DLLAPI CNOID_OMPL_DLLIMPORT
#  endif // CNOID_OMPL_EXPORTS
#  define CNOID_OMPL_LOCAL CNOID_OMPL_DLLLOCAL
# endif // CNOID_OMPL_STATIC

#endif //! CNOID_OMPL_PLUGIN_EXPORTDECL_H

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_OMPL_DLLAPI
