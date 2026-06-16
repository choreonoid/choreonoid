#ifndef CNOID_UTIL_URI_SCHEME_PROCESSOR_H
#define CNOID_UTIL_URI_SCHEME_PROCESSOR_H

#include <string>
#include <vector>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class FilePathVariableProcessor;

class CNOID_EXPORT UriSchemeProcessor
{
public:
    /**
       Signature of a URI scheme handler. The handler is given the URI's scheme-less path
       and a reference to the processor instance that invoked it. The processor exposes the
       caller's context (base directory, model search directories, ...), so handlers can be
       written without depending on any thread-local state or global storage. The handler
       returns the resolved file path on success, or an empty string on failure. On failure
       the handler should call processor.setErrorMessage() so that the caller can retrieve
       the diagnostic via UriSchemeProcessor::errorMessage().
    */
    typedef std::function<std::string(const std::string& path,
                                      UriSchemeProcessor& processor)> UriSchemeHandler;

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

    /**
       Adds a directory that registered URI scheme handlers may consult when resolving a URI
       that identifies a resource by name (e.g. "model://<name>/..."). The interpretation is
       left to each handler. Directories are kept in registration order and the previous
       contents are preserved across calls.
    */
    void addModelSearchDirectory(const std::string& directory);

    void clearModelSearchDirectories();

    const std::vector<std::string>& modelSearchDirectories() const;

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

    /**
       Adds a diagnostic message describing why URI resolution failed. Registered URI scheme
       handlers call this on failure so that the caller (which sees getFilePath() returning
       an empty string) can retrieve the reason via errorMessage() and route it to its own
       logging or exception machinery. Multiple calls within the same getFilePath()
       invocation are concatenated with newline separators so that diagnostics from
       successive failure sources are all preserved. The accumulated message is cleared at
       the start of each getFilePath() call.
    */
    void setErrorMessage(const std::string& message);

    /**
       Returns the accumulated diagnostic for the most recent getFilePath() call, or an
       empty string if that call succeeded or no message was set. Multi-line content
       indicates several failure points reported within the same resolution attempt.
    */
    std::string errorMessage() const;

private:
    class Impl;
    Impl* impl;
};

}

#endif
