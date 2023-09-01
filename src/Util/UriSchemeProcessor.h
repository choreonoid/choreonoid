#ifndef CNOID_UTIL_URI_SCHEME_PROCESSOR_H
#define CNOID_UTIL_URI_SCHEME_PROCESSOR_H

#include <iosfwd>
#include <string>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class FilePathVariableProcessor;

class CNOID_EXPORT UriSchemeProcessor
{
public:
    typedef std::function<std::string(const std::string& path, std::ostream& os)> UriSchemeHandler;

    static void registerUriSchemeHandler(const std::string& scheme, UriSchemeHandler handler);

    UriSchemeProcessor();
    ~UriSchemeProcessor();

    /**
       If a FilePathVariableProcessor object is set, the getFilePath function returns the path
       where path variables are expanded by the FilePathVariableProcessor.
    */
    void setFilePathVariableProcessor(FilePathVariableProcessor* processor);

    FilePathVariableProcessor* filePathVariableProcessor();
    
    /**
       If a base directory is specified, the getFilePath function returns the absolute path
       from the base directory when the path included in the URI is a relative path. Otherwise,
       the getFilePath function left the relative path.
       Note that the base directory specified by this function is not processed when a
       FilePathVariableProcessor is set by the setFilePathVariableProcessor function
       and the base directory specified in the FilePathVariableProcessor is processed.
    */
    void setBaseDirectory(const std::string& directory);

    /**
       This function sets the base directory of the given filename. The other specifications of
       this function are the same as those of the setBaseDirectory function.
    */
    void setBaseDirectoryFor(const std::string& filename);

    std::string baseDirectory() const;

    bool detectScheme(const std::string& uri);
    std::string getFilePath(const std::string& uri);

    /**
       If a FilePathVariableProcessor object is set, this function returns a file path string
       that may be parameterized by the FilePathVariableProcessor.
    */
    std::string getParameterizedFilePath(const std::string& uri);

    const std::string& scheme() const;
    bool isFileScheme() const;
    bool isSupportedScheme() const;
    const std::string& path();
    std::string errorMessage() const;

private:
    class Impl;
    Impl* impl;
};

}

#endif
