/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EXECUTABLE_PATH_H
#define CNOID_UTIL_EXECUTABLE_PATH_H

#include <string>
#include <cnoid/Config>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT const std::string& executablePath();
CNOID_EXPORT const std::string& executableDirectory();
CNOID_EXPORT const std::string& executableTopDirectory();
CNOID_EXPORT const std::string& pluginDirectory();
CNOID_EXPORT const std::string& shareDirectory();
CNOID_EXPORT const std::string& executableBasename();

}

#endif
