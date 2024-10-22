#ifndef CNOID_UTIL_COMPILER_WARNING_CONTROL_MACROS_H
#define CNOID_UTIL_COMPILER_WARNING_CONTROL_MACROS_H

#ifdef __GNUC__
    #define CNOID_DISABLE_DEPRECATED_WARNINGS \
        _Pragma("GCC diagnostic push") \
        _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"") \
        _Pragma("GCC diagnostic ignored \"-Wdeprecated-enum-enum-conversion\"")
    #define CNOID_RESTORE_WARNINGS _Pragma("GCC diagnostic pop")
#elif defined(_MSC_VER)
    #define CNOID_DISABLE_DEPRECATED_WARNINGS \
        __pragma(warning(push)) \
        __pragma(warning(disable: 4996)) \
        __pragma(warning(disable: 5054))
    #define CNOID_RESTORE_WARNINGS __pragma(warning(pop))
#else
    #define CNOID_DISABLE_DEPRECATED_WARNINGS
    #define CNOID_RESTORE_WARNINGS
#endif

#endif
