/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EXECUTABLE_PATH_H
#define CNOID_UTIL_EXECUTABLE_PATH_H

#include <string>
#include <cnoid/stdx/filesystem>
#include <cnoid/Config>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT const std::string& executableFile();
CNOID_EXPORT const std::string& executableBasename();
CNOID_EXPORT const std::string& executableDir();
CNOID_EXPORT const std::string& executableTopDir();
CNOID_EXPORT stdx::filesystem::path executableTopDirPath();
CNOID_EXPORT const std::string& pluginDir();
CNOID_EXPORT stdx::filesystem::path pluginDirPath();
CNOID_EXPORT const std::string& shareDir();
CNOID_EXPORT stdx::filesystem::path shareDirPath();

[[deprecated("Use cnoid::executableFile().")]]
inline const std::string& executablePath() { return executableFile(); }

[[deprecated("Use cnoid::executableDir().")]]
inline const std::string& executableDirectory() { return executableDir(); }

[[deprecated("Use cnoid::executableTopDir().")]]
inline const std::string& executableTopDirectory() { return executableTopDir(); }

[[deprecated("Use cnoid::pluginDir().")]]
inline const std::string& pluginDirectory() { return pluginDir(); }

[[deprecated("Use cnoid::shareDir().")]]
inline const std::string& shareDirectory() { return shareDir(); }

}

#endif
