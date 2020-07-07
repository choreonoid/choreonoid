/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_FILE_PATH_VARIABLE_PROCESSOR_H
#define CNOID_UTIL_FILE_PATH_VARIABLE_PROCESSOR_H

#include "Referenced.h"
#include <cnoid/stdx/filesystem>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT FilePathVariableProcessor : public Referenced
{
public:
    static FilePathVariableProcessor* systemInstance();
    
    FilePathVariableProcessor();
    ~FilePathVariableProcessor();

    void setSystemVariablesEnabled(bool on);
    void setUserVariables(Mapping* variables);
    void setBaseDirectory(const std::string& directory);
    void clearBaseDirectory();
    std::string baseDirectory() const;
    stdx::filesystem::path baseDirPath() const;
    void setProjectDirectory(const std::string& directory);
    void clearProjectDirectory();
    const std::string& projectDirectory() const;
    std::string parameterize(const std::string& path);
    std::string expand(const std::string& path, bool doMakeNativeAbsolutePath);
    const std::string& errorMessage() const;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<FilePathVariableProcessor> FilePathVariableProcessorPtr;

}

#endif
