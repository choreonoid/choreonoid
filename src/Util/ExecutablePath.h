/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EXECUTABLE_PATH_H_INCLUDED
#define CNOID_UTIL_EXECUTABLE_PATH_H_INCLUDED

#include <string>
#include "Config.h"
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT const std::string& executablePath();
CNOID_EXPORT const std::string& executableTopDirectory();
CNOID_EXPORT const std::string& shareDirectory();
CNOID_EXPORT const std::string& executableBasename();
}

#endif
