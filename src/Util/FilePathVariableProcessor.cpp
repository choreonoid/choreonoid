#include "FilePathVariableProcessor.h"
#include "ValueTree.h"
#include "ExecutablePath.h"
#include "FileUtil.h"
#include "UTF8.h"
#include <cnoid/stdx/optional>
#include <fmt/format.h>
#include <regex>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = stdx::filesystem;

namespace cnoid {

ref_ptr<FilePathVariableProcessor> FilePathVariableProcessor::currentInstance_;

class FilePathVariableProcessor::Impl
{
public:
    stdx::optional<regex> variableRegex;
    filesystem::path baseDirPath;
    filesystem::path projectDirPath;
    string projectDirString;
    bool isProjectDirDifferentFromBaseDir;

    bool isSubstitutionWithSystemPathVariableEnabled;
    filesystem::path topDirPath;
    string topDirString;
    filesystem::path shareDirPath;
    string shareDirString;
    filesystem::path homeDirPath;
    string homeDirString;

    enum VarType { Base, Project, User, AppSpecific, Share, Top, Home, NumVarTypes };

    struct VarTypeInfo
    {
        string varName;
        filesystem::path relativePath;
        int relativePathLength;
    };

    VarTypeInfo varTypeInfos[NumVarTypes];

    typedef unordered_map<string, std::vector<filesystem::path>> VarNameToPathListMap;
    VarNameToPathListMap appSpecificVariables;
    VarNameToPathListMap userVariables;
    
    string errorMessage;

    Impl();
    Impl(const Impl& org);
    std::string parameterize(const std::string& orgPathString);
    bool findSubPathForPathVariable(
        const filesystem::path& path, int varType, const filesystem::path& dirPath, int additionalPathLength = 0);
    void findSubPathForPathVariables(
        const filesystem::path& path, int varType, const VarNameToPathListMap& variables);
    int getPathLength(filesystem::path& path, int additionalLength = 0);
    bool replaceUserVariable(
        string& io_pathString, const string& varname, int pos, int len, const VarNameToPathListMap& variables);
    std::string expand(const std::string& pathString, bool doMakeNativeAbsolutePath);
};

}


FilePathVariableProcessor* FilePathVariableProcessor::systemInstance()
{
    static FilePathVariableProcessorPtr instance;

    if(!instance){
        instance = new FilePathVariableProcessor;
        instance->setSubstitutionWithSystemPathVariableEnabled(true);
    }

    return instance;
}


FilePathVariableProcessor::FilePathVariableProcessor()
{
    impl = new Impl;
}


FilePathVariableProcessor::Impl::Impl()
{
    isProjectDirDifferentFromBaseDir = false;
    isSubstitutionWithSystemPathVariableEnabled = false;

    topDirPath = executableTopDirPath();
    topDirString = executableTopDir();
    varTypeInfos[Top].varName = "PROGRAM_TOP";

    shareDirPath = cnoid::shareDirPath();
    shareDirString = shareDir();
    varTypeInfos[Share].varName = "SHARE";

    char* home = getenv("HOME");
    if(home){
        homeDirPath = string(home);
        homeDirString = toUTF8(home);
    }
    varTypeInfos[Home].varName = "HOME";

    varTypeInfos[Project].varName = "PROJECT_DIR";
}


FilePathVariableProcessor::FilePathVariableProcessor(const FilePathVariableProcessor& org)
{
    impl = new Impl(*org.impl);
}


FilePathVariableProcessor::Impl::Impl(const Impl& org)
    : isProjectDirDifferentFromBaseDir(false),
      isSubstitutionWithSystemPathVariableEnabled(org.isSubstitutionWithSystemPathVariableEnabled),
      topDirPath(org.topDirPath),
      topDirString(org.topDirString),
      shareDirPath(org.shareDirPath),
      shareDirString(org.shareDirString),
      homeDirPath(org.homeDirPath),
      homeDirString(org.homeDirString),
      appSpecificVariables(org.appSpecificVariables),
      userVariables(org.userVariables)
{
    varTypeInfos[Top].varName = "PROGRAM_TOP";
    varTypeInfos[Share].varName = "SHARE";
    varTypeInfos[Home].varName = "HOME";
    varTypeInfos[Project].varName = "PROJECT_DIR";
}


FilePathVariableProcessor::~FilePathVariableProcessor()
{
    delete impl;
}


void FilePathVariableProcessor::setSubstitutionWithSystemPathVariableEnabled(bool on)
{
    impl->isSubstitutionWithSystemPathVariableEnabled = on;
}


void FilePathVariableProcessor::addAppSpecificVariable(const std::string& name, const stdx::filesystem::path& path)
{
    impl->appSpecificVariables[name].emplace_back(path);
}


void FilePathVariableProcessor::addUserVariable(const std::string& name, const stdx::filesystem::path& path)
{
    impl->userVariables[name].emplace_back(path);
}


void FilePathVariableProcessor::clearUserVariables()
{
    impl->userVariables.clear();
}


std::vector<std::pair<std::string, stdx::filesystem::path>> FilePathVariableProcessor::getUserVariables() const
{
    std::vector<std::pair<std::string, stdx::filesystem::path>> variables;
    variables.reserve(impl->userVariables.size());
    for(auto& kv : impl->userVariables){
        auto& name = kv.first;
        auto& paths = kv.second;
        for(auto& path : paths){
            variables.emplace_back(make_pair(name, path));
        }
    }
    return variables;
}


void FilePathVariableProcessor::storeUserVariables(Mapping* variables)
{
    variables->clear();
    for(auto& kv : impl->userVariables){
        auto& name = kv.first;
        auto& paths = kv.second;
        if(!paths.empty()){
            auto list = variables->createListing(name);
            for(auto& path : paths){
                list->append(path.generic_string(), DOUBLE_QUOTED);
            }
        }
    }
}


void FilePathVariableProcessor::restoreUserVariables(Mapping* variables)
{
    impl->userVariables.clear();
    for(auto& kv : *variables){
        auto& name = kv.first;
        auto node = kv.second;
        if(node->isListing()){
            auto list = node->toListing();
            if(!list->empty()){
                auto& paths = impl->userVariables[name];
                paths.reserve(list->size());
                for(auto& node : *list){
                    if(node->isString()){
                        filesystem::path path(node->toString());
                        if(path.is_absolute()){
                            paths.push_back(path);
                        }
                    }
                }
            }
        }
    }
}


void FilePathVariableProcessor::setBaseDirectory(const std::string& directory)
{
    setBaseDirPath(fromUTF8(directory));
}


void FilePathVariableProcessor::setBaseDirPath(const stdx::filesystem::path& path)
{
    impl->baseDirPath = path;
    if(impl->baseDirPath.is_relative()){
        impl->baseDirPath = filesystem::current_path() /  impl->baseDirPath;
    }
    impl->isProjectDirDifferentFromBaseDir = (impl->baseDirPath != impl->projectDirPath);
}


void FilePathVariableProcessor::clearBaseDirectory()
{
    setBaseDirPath("");
}


std::string FilePathVariableProcessor::baseDirectory() const
{
    return toUTF8(impl->baseDirPath.string());
}


const stdx::filesystem::path& FilePathVariableProcessor::baseDirPath() const
{
    return impl->baseDirPath;
}


void FilePathVariableProcessor::setProjectDirectory(const std::string& directory)
{
    setProjectDirPath(fromUTF8(directory));
}


void FilePathVariableProcessor::setProjectDirPath(const stdx::filesystem::path& path)
{
    impl->projectDirPath = path;
    if(impl->projectDirPath.is_relative()){
        impl->projectDirPath = filesystem::current_path() /  impl->projectDirPath;
    }
    impl->projectDirString = toUTF8(impl->projectDirPath.generic_string());

    impl->isProjectDirDifferentFromBaseDir = (impl->baseDirPath != impl->projectDirPath);
}


void FilePathVariableProcessor::clearProjectDirectory()
{
    setProjectDirectory("");
}


const std::string& FilePathVariableProcessor::projectDirectory() const
{
    return impl->projectDirString;
}


const stdx::filesystem::path& FilePathVariableProcessor::projectDirPath() const
{
    return impl->projectDirPath;
}


std::string FilePathVariableProcessor::parameterize(const std::string& orgPathString)
{
    return impl->parameterize(orgPathString);
}


std::string FilePathVariableProcessor::Impl::parameterize(const std::string& orgPathString)
{
    string output;
    
    filesystem::path orgPath(fromUTF8(orgPathString));

    if(orgPath.is_absolute()){
        for(int i=0; i < NumVarTypes; ++i){
            varTypeInfos[i].relativePathLength = std::numeric_limits<int>::max();
        }
        if(!baseDirPath.empty()){
            findSubPathForPathVariable(orgPath, Base, baseDirPath);
        }
        if(isProjectDirDifferentFromBaseDir){
            findSubPathForPathVariable(orgPath, Project, projectDirPath);
        }
        findSubPathForPathVariables(orgPath, User, userVariables);
        findSubPathForPathVariables(orgPath, AppSpecific, appSpecificVariables);
        
        if(isSubstitutionWithSystemPathVariableEnabled){
            findSubPathForPathVariable(orgPath, Share, shareDirPath);
            findSubPathForPathVariable(orgPath, Top, topDirPath);

            /*
            if(!homeDirPath.empty()){
                findSubPathForPathVariable(orgPath, Home, homeDirPath, 1);
            }
            */
        }

        int minLength = std::numeric_limits<int>::max();
        int index = -1;
        for(int i=0; i < NumVarTypes; ++i){
            int length = varTypeInfos[i].relativePathLength;
            if(length < minLength){
                minLength = length;
                index = i;
            }
        }

        if(index < 0){
            // Check if a relative path including ".." from the base directory is accepted
            // or the path is a sub path of the home directory.
            if(auto fromBase = getRelativePath(orgPath, baseDirPath)){
                auto commonPath = baseDirPath;
                auto it = fromBase->begin();
                while(it != fromBase->end()){
                    if(*it == ".."){
                        commonPath = commonPath.parent_path();
                        ++it;
                    } else {
                        break;
                    }
                }
                if(!homeDirPath.empty()){
                    if(auto homeToCommon = getRelativePath(commonPath, homeDirPath)){
                        if(homeToCommon->empty()){
                            if(findSubPathForPathVariable(orgPath, Home, homeDirPath)){
                                index = Home;
                            }
                        }
                    }
                }
                if(index < 0){
                    index = Base;
                    auto& info = varTypeInfos[Base];
                    info.relativePath = *fromBase;
                    minLength = getPathLength(*fromBase);
                }
            }
        }
        
        if(index >= 0){
            auto& info = varTypeInfos[index];
            if(index == Base){
                if(getPathLength(orgPath) < minLength){
                    output = toUTF8(orgPath.generic_string()); // absolute path
                } else {
                    output = toUTF8(info.relativePath.generic_string());
                }
            } else {
                output = format("${{{0}}}/{1}", info.varName, toUTF8(info.relativePath.generic_string()));
            }
        }
    }

    if(output.empty()){
        output = toUTF8(orgPath.generic_string()); // absolute path
    }

    return output;
}


bool FilePathVariableProcessor::Impl::findSubPathForPathVariable
(const filesystem::path& path, int varType, const filesystem::path& dirPath, int additionalPathLength)
{
    auto& info = varTypeInfos[varType];
    if(findPathInDirectory(dirPath, path, info.relativePath)){
        info.relativePathLength = getPathLength(info.relativePath, additionalPathLength);
        return true;
    }
    return false;
}


void FilePathVariableProcessor::Impl::findSubPathForPathVariables
(const filesystem::path& path, int varType, const VarNameToPathListMap& variables)
{
    auto& info = varTypeInfos[varType];
    info.relativePath.clear();
    filesystem::path relativePath;
    int maxMatchedDepth = 0;
    for(auto& kv : variables){
        auto& dirPaths = kv.second;
        for(auto& dirPath : dirPaths){
            int n = findPathInDirectory(dirPath, path, relativePath);
            if(n > maxMatchedDepth){
                maxMatchedDepth = n;
                info.relativePath = relativePath;
                info.varName = kv.first;
            }
        }
    }
    if(maxMatchedDepth > 0){
        info.relativePathLength = getPathLength(info.relativePath);
    }
}


int FilePathVariableProcessor::Impl::getPathLength(filesystem::path& path, int additionalLength)
{
    int length = additionalLength;
    bool hasUpElement = false;
    auto iter = path.begin();
    while(iter != path.end()){
        if(*iter == ".."){
            hasUpElement = true;
        }
        ++length;
        ++iter;
    }
    if(hasUpElement){
        ++length;
    }
    return length;
}


std::string FilePathVariableProcessor::expand
(const std::string& pathString, bool doMakeNativeAbsolutePath)
{
    return impl->expand(pathString, doMakeNativeAbsolutePath);
}


std::string FilePathVariableProcessor::Impl::expand
(const std::string& pathString, bool doMakeNativeAbsolutePath)
{
    if(!variableRegex){
        variableRegex.emplace("^\\$\\{(\\w+)\\}");
    }

    bool expansionFailed = false;
    string expanded = pathString;
    smatch match;

    if(regex_search(pathString, match, *variableRegex)){
        int pos = match.position();
        int len = match.length();
        string varname = match.str(1);

        if(varname == "SHARE"){
            expanded.replace(pos, len, shareDirString);
            
        } else if(varname == "PROGRAM_TOP"){
            expanded.replace(pos, len, topDirString);
            
        } else if(varname == "PROJECT_DIR"){
            if(projectDirString.empty()){
                errorMessage = format(_("${{PROJECT_DIR}} of \"{0}\" cannot be expanded."), pathString);
                expansionFailed = true;
                expanded.clear();
            } else {
                expanded.replace(pos, len, projectDirString);
            }
            
        } else if(varname == "HOME"){
            expanded.replace(pos, len, homeDirString);
            
        } else {
            if(!replaceUserVariable(expanded, varname, pos, len, userVariables)){
                if(!replaceUserVariable(expanded, varname, pos, len, appSpecificVariables)){
                    expansionFailed = true;
                    expanded.clear();
                    errorMessage =
                        format(_("Path variable \"{0}\" in \"{1}\" is not defined."),
                               varname, pathString);
                }
            }
        }
    }

    if(expansionFailed || !doMakeNativeAbsolutePath){
        return expanded;
    }
    
    filesystem::path path(fromUTF8(expanded));
    if(!path.is_absolute()){
        if(!baseDirPath.empty()){
            path = baseDirPath / path;
        } else {
            path = filesystem::current_path() / path;
        }
    }
    path = filesystem::lexically_normal(path);
    return toUTF8(path.make_preferred().string());
}


bool FilePathVariableProcessor::Impl::replaceUserVariable
(string& io_pathString, const string& varName, int pos, int len, const VarNameToPathListMap& variables)
{
    auto it = variables.find(varName);
    if(it == variables.end()){
        return false;
    }
    auto& paths = it->second;
    if(paths.empty()){
        return false;
    }
    
    string replaced(io_pathString);
    replaced.replace(pos, len, paths.front().string());

    if(paths.size() >= 2){
        if(!filesystem::exists(fromUTF8(replaced))){
            string replaced2;
            for(size_t i=1; i < paths.size(); ++i){
                replaced2 = io_pathString;
                replaced2.replace(pos, len, paths[i].string());
                if(filesystem::exists(fromUTF8(replaced2))){
                    replaced = std::move(replaced2);
                    break;
                }
            }
        }
    }

    io_pathString = replaced;
    return true;
}


const std::string& FilePathVariableProcessor::errorMessage() const
{
    return impl->errorMessage;
}
