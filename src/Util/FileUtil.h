#ifndef CNOID_UTIL_FILE_UTIL_H
#define CNOID_UTIL_FILE_UTIL_H

#include <filesystem>
#include <optional>
#include <string>
#include "exportdecl.h"

namespace cnoid {

extern CNOID_EXPORT const char* DLL_PREFIX;
extern CNOID_EXPORT const char* DLL_SUFFIX;
extern CNOID_EXPORT const char* DLL_EXTENSION;
extern CNOID_EXPORT const char* EXEC_SUFFIX;
extern CNOID_EXPORT const char* EXEC_EXTENSION;
extern CNOID_EXPORT const char* PATH_DELIMITER;


[[deprecated("Use path.lexically_normal().")]]
CNOID_EXPORT std::filesystem::path getCompactPath(const std::filesystem::path& path);

[[deprecated("Use path.lexically_normal().")]]
CNOID_EXPORT void makePathCompact(std::filesystem::path& io_path);


CNOID_EXPORT int findPathInDirectory(
    const std::filesystem::path& directory,
    const std::filesystem::path& path,
    std::filesystem::path& out_relativePath);

[[deprecated("Use findPathInDirectory.")]]
CNOID_EXPORT int findSubDirectory(
    const std::filesystem::path& directory,
    const std::filesystem::path& path,
    std::filesystem::path& out_subdirectory);

/**
   \return Empty optional value is returned if a relative path cannot be obtained for the input path set, empty value.
   Otherwise, the optional value containing the relative path is returned.
*/
CNOID_EXPORT std::optional<std::filesystem::path> getRelativePath(
    const std::filesystem::path& path, const std::filesystem::path& base);

CNOID_EXPORT bool checkIfSubFilePath(const std::filesystem::path& path, const std::filesystem::path& base);

[[deprecated("Use getRelativePath.")]]
CNOID_EXPORT bool findRelativePath(
    const std::filesystem::path& from,
    const std::filesystem::path& to,
    std::filesystem::path& out_relativePath);

CNOID_EXPORT std::filesystem::path getNativeUniformPath(const std::filesystem::path& path);

/**
   This function returs the extension without dot.
*/
[[deprecated("Use path.extension().string(). Note that the dot character is added to the beginning of the string.")]]
CNOID_EXPORT std::string getExtension(const std::filesystem::path& path);

[[deprecated("Use path.generic_string().")]]
CNOID_EXPORT std::string getGenericPathString(const std::filesystem::path& path);

[[deprecated("Use path.isAbsolute().")]]
CNOID_EXPORT bool checkAbsolute(const std::filesystem::path& path);

[[deprecated("Use filesystem::absolute(path).")]]
CNOID_EXPORT std::filesystem::path getAbsolutePath(const std::filesystem::path& path);

[[deprecated("Use filesystem::absolute(path).string().")]]
CNOID_EXPORT std::string getAbsolutePathString(const std::filesystem::path& path);

[[deprecated("Use path.filename().string().")]]
CNOID_EXPORT std::string getFilename(const std::filesystem::path& path);

[[deprecated("Make a filesystem::path object and use filename().string().")]]
CNOID_EXPORT std::string getFilename(const std::string& pathString);

[[deprecated("Use path.stem().string().")]]
CNOID_EXPORT std::string getBasename(const std::filesystem::path& path);

[[deprecated("Use path.string().")]]
CNOID_EXPORT std::string getPathString(const std::filesystem::path& path);

[[deprecated("Use path.make_preferred().string().")]]
CNOID_EXPORT std::string getNativePathString(const std::filesystem::path& path);

[[deprecated("Use the original string value passed as an argument.")]]
CNOID_EXPORT std::string toActualPathName(const std::string& pathName);

}
    
#endif
