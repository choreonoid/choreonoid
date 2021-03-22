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


[[deprecated("Use filesystem::lexically_normal(path).")]]
CNOID_EXPORT stdx::filesystem::path getCompactPath(const stdx::filesystem::path& path);

[[deprecated("Use filesystem::lexically_normal(path).")]]
CNOID_EXPORT void makePathCompact(stdx::filesystem::path& io_path);


CNOID_EXPORT int findSubDirectory(
    const stdx::filesystem::path& directory,
    const stdx::filesystem::path& path,
    stdx::filesystem::path& out_subdirectory);

CNOID_EXPORT bool findRelativePath(
    const stdx::filesystem::path& from,
    const stdx::filesystem::path& to,
    stdx::filesystem::path& out_relativePath);


/**
   This function returs the extension without dot.
*/
[[deprecated("Use path.extension().string(). Note that the dot character is added to the beginning of the string.")]]
CNOID_EXPORT std::string getExtension(const stdx::filesystem::path& path);

[[deprecated("Use path.generic_string().")]]
CNOID_EXPORT std::string getGenericPathString(const stdx::filesystem::path& path);

[[deprecated("Use path.isAbsolute().")]]
CNOID_EXPORT bool checkAbsolute(const stdx::filesystem::path& path);

[[deprecated("Use filesystem::absolute(path).")]]
CNOID_EXPORT stdx::filesystem::path getAbsolutePath(const stdx::filesystem::path& path);

[[deprecated("Use filesystem::absolute(path).string().")]]
CNOID_EXPORT std::string getAbsolutePathString(const stdx::filesystem::path& path);

[[deprecated("Use path.filename().string().")]]
CNOID_EXPORT std::string getFilename(const stdx::filesystem::path& path);

[[deprecated("Make a filesystem::path object and use filename().string().")]]
CNOID_EXPORT std::string getFilename(const std::string& pathString);

[[deprecated("Use path.stem().string().")]]
CNOID_EXPORT std::string getBasename(const stdx::filesystem::path& path);

[[deprecated("Use path.string().")]]
CNOID_EXPORT std::string getPathString(const stdx::filesystem::path& path);

[[deprecated("Use path.make_preferred().string().")]]
CNOID_EXPORT std::string getNativePathString(const stdx::filesystem::path& path);

[[deprecated("Use the original string value passed as an argument.")]]
CNOID_EXPORT std::string toActualPathName(const std::string& pathName);

}
    
#endif
