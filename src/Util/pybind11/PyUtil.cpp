#if defined _WIN32
#include "PyUtil.h"

//In Visual Studio, if there is no expored function, the lib file not be created.
namespace {
    __declspec(dllexport) void dummy_function(){};
}
#endif