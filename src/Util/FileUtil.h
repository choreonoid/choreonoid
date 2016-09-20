/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_FILE_UTIL_H
#define CNOID_UTIL_FILE_UTIL_H

#include <boost/filesystem.hpp>
#include <string>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT void makePathCompact(
    const boost::filesystem::path& path,
    boost::filesystem::path& out_compact);

CNOID_EXPORT int findSubDirectory(
    const boost::filesystem::path& directory,
    const boost::filesystem::path& path,
    boost::filesystem::path& out_subdirectory);

CNOID_EXPORT bool findRelativePath(
    const boost::filesystem::path& from,
    const boost::filesystem::path& to,
    boost::filesystem::path& out_relativePath);

CNOID_EXPORT std::string getExtension(const boost::filesystem::path& path);

/*
   The following functions were originally defined to support both the version 2 and 3 of
   the boost.filesystem library. However, supporting the version 2 was stopped, and the use
   of these functions should be replaced with the original functions of the version 3.
*/
CNOID_EXPORT std::string getGenericPathString(const boost::filesystem::path& path);
CNOID_EXPORT bool checkAbsolute(const boost::filesystem::path& path);
CNOID_EXPORT boost::filesystem::path getAbsolutePath(const boost::filesystem::path& path);
CNOID_EXPORT std::string getAbsolutePathString(const boost::filesystem::path& path);
CNOID_EXPORT std::string getFilename(const boost::filesystem::path& path);
CNOID_EXPORT std::string getFilename(const std::string& pathString);
CNOID_EXPORT std::string getBasename(const boost::filesystem::path& path);
CNOID_EXPORT std::string getPathString(const boost::filesystem::path& path);
CNOID_EXPORT std::string getNativePathString(const boost::filesystem::path& path);

CNOID_EXPORT std::string toActualPathName(const std::string& pathName);

}
    
#endif
