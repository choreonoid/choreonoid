#ifndef CNOID_UTIL_ZIP_ARCHIVER_H
#define CNOID_UTIL_ZIP_ARCHIVER_H

#include "exportdecl.h"
#include <string>
#include <vector>

namespace cnoid {

class CNOID_EXPORT ZipArchiver
{
public:
    ZipArchiver();
    virtual ~ZipArchiver();

    bool createZipFile(const std::string& zipFilename, const std::string& directory);
    bool extractZipFile(const std::string& zipFilename, const std::string& directory);
    const std::vector<std::string>& extractedFiles() const;

    enum ErrorType {
        NoError,
        ExistingZipFileRemovalError,
        ZipFileCreationError,
        DirectoryAdditionError,
        FileAdditionError,
        ZipFileOpenError,
        EntryExtractionError,
        DirectoryCreationError,
        FileExtractionError
    };

    ErrorType errorType() const;
    const std::string& systemErrorMessage() const;
    const std::string& errorMessage() const;

private:
    class Impl;
    Impl* impl;
};

}

#endif 
