#include <cnoid/Config>

#ifdef CNOID_ENABLE_GPERFTOOLS_PROFILER

#include <gperftools/profiler.h>

void dummy_function_to_link_profiler_library()
{
    ProfilerState state;
    ProfilerGetCurrentState(&state);
}

#endif
