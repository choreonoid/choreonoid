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

    static FilePathVariableProcessor* currentInstance() {
        return currentInstance_;
    }
    static void setCurrentInstance(FilePathVariableProcessor* instance){
        currentInstance_ = instance;
    }
    
    FilePathVariableProcessor();
    FilePathVariableProcessor(const FilePathVariableProcessor& org);
    ~FilePathVariableProcessor();

    void setSubstitutionWithSystemPathVariableEnabled(bool on);
    void addAppSpecificVariable(const std::string& name, const stdx::filesystem::path& path);
    void addUserVariable(const std::string& name, const stdx::filesystem::path& path);
    std::vector<std::pair<std::string, stdx::filesystem::path>> getUserVariables() const;
    void clearUserVariables();
    void storeUserVariables(Mapping* variables);
    void restoreUserVariables(Mapping* variables);
    void setBaseDirectory(const std::string& directory);
    void setBaseDirPath(const stdx::filesystem::path& path);
    void clearBaseDirectory();
    std::string baseDirectory() const;
    const stdx::filesystem::path& baseDirPath() const;
    void setProjectDirectory(const std::string& directory);
    void setProjectDirPath(const stdx::filesystem::path& path);
    void clearProjectDirectory();
    const std::string& projectDirectory() const;
    const stdx::filesystem::path& projectDirPath() const;
    std::string parameterize(const std::string& path);
    std::string expand(const std::string& path, bool doMakeNativeAbsolutePath);
    const std::string& errorMessage() const;

private:
    static ref_ptr<FilePathVariableProcessor> currentInstance_;
    
    class Impl;
    Impl* impl;
};

typedef ref_ptr<FilePathVariableProcessor> FilePathVariableProcessorPtr;

}

#endif
