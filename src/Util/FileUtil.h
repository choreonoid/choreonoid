/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_FILE_UTIL_H
#define CNOID_UTIL_FILE_UTIL_H

#include <cnoid/stdx/filesystem>
#include <string>
#include "exportdecl.h"

namespace cnoid {

extern CNOID_EXPORT const char* DLL_PREFIX;
extern CNOID_EXPORT const char* DLL_SUFFIX;
extern CNOID_EXPORT const char* DLL_EXTENSION;
extern CNOID_EXPORT const char* EXEC_SUFFIX;
extern CNOID_EXPORT const char* EXEC_EXTENSION;
extern CNOID_EXPORT const char* PATH_DELIMITER;

CNOID_EXPORT stdx::filesystem::path getCompactPath(const stdx::filesystem::path& path);
CNOID_EXPORT void makePathCompact(stdx::filesystem::path& io_path);


CNOID_EXPORT int findSubDirectory(
    const stdx::filesystem::path& directory,
    const stdx::filesystem::path& path,
    stdx::filesystem::path& out_subdirectory);

CNOID_EXPORT bool findRelativePath(
    const stdx::filesystem::path& from,
    const stdx::filesystem::path& to,
    stdx::filesystem::path& out_relativePath);

CNOID_EXPORT std::string getExtension(const stdx::filesystem::path& path);

/*
   The following functions were originally defined to support both the version 2 and 3 of
   the boost.filesystem library. However, supporting the version 2 was stopped, and the use
   of these functions should be replaced with the original functions of the version 3.
*/
CNOID_EXPORT std::string getGenericPathString(const stdx::filesystem::path& path);
CNOID_EXPORT bool checkAbsolute(const stdx::filesystem::path& path);
CNOID_EXPORT stdx::filesystem::path getAbsolutePath(const stdx::filesystem::path& path);
CNOID_EXPORT std::string getAbsolutePathString(const stdx::filesystem::path& path);
CNOID_EXPORT std::string getFilename(const stdx::filesystem::path& path);
CNOID_EXPORT std::string getFilename(const std::string& pathString);
CNOID_EXPORT std::string getBasename(const stdx::filesystem::path& path);
CNOID_EXPORT std::string getPathString(const stdx::filesystem::path& path);
CNOID_EXPORT std::string getNativePathString(const stdx::filesystem::path& path);

CNOID_EXPORT std::string toActualPathName(const std::string& pathName);

}
    
#endif
