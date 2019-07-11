#ifndef CNOID_UTIL_STRING_UTIL_H
#define CNOID_UTIL_STRING_UTIL_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT void trim(std::string& s);
CNOID_EXPORT std::string trimmed(const std::string& s);

}

#endif
