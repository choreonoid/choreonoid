/**
   @author Shin'ichiro Nakaoka
*/

#include "FilePathVariableProcessor.h"
#include "ValueTree.h"
#include "ExecutablePath.h"
#include "FileUtil.h"
#include "UTF8.h"
#include <cnoid/stdx/optional>
#include <fmt/format.h>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = stdx::filesystem;

namespace cnoid {

class FilePathVariableProcessor::Impl
{
public:
    MappingPtr userVariables;
    stdx::optional<regex> variableRegex;
    filesystem::path baseDirPath;
    bool isBaseDirInHome;
    filesystem::path projectDirPath;
    string projectDirString;

    bool isSystemVariableSetEnabled;
    filesystem::path topDirPath;
    string topDirString;
    filesystem::path shareDirPath;
    string shareDirString;
    filesystem::path homeDirPath;
    string homeDirString;
    
    string errorMessage;

    Impl();
    void setSystemVariablesEnabled(bool on);
    std::string parameterize(const std::string& orgPathString);
    bool findSubDirectoryOfDirectoryVariable(
        const filesystem::path& path, std::string& out_varName, filesystem::path& out_relativePath);
    bool replaceUserVariable(string& io_pathString, const string& varname, int pos, int len);
    std::string expand(const std::string& pathString, bool doMakeNativeAbsolutePath);
};

}


FilePathVariableProcessor* FilePathVariableProcessor::systemInstance()
{
    static FilePathVariableProcessorPtr instance;

    if(!instance){
        instance = new FilePathVariableProcessor;
        instance->setSystemVariablesEnabled(true);
    }

    return instance;
}


FilePathVariableProcessor::FilePathVariableProcessor()
{
    impl = new Impl;
}


FilePathVariableProcessor::Impl::Impl()
{
    isSystemVariableSetEnabled = false;
    isBaseDirInHome = false;
}


FilePathVariableProcessor::~FilePathVariableProcessor()
{
    delete impl;
}


void FilePathVariableProcessor::setSystemVariablesEnabled(bool on)
{
    impl->setSystemVariablesEnabled(on);
}


void FilePathVariableProcessor::Impl::setSystemVariablesEnabled(bool on)
{
    isSystemVariableSetEnabled = on;
    
    if(on){
        topDirString = executableTopDirectory();
        topDirPath = topDirString;

        shareDirString = shareDirectory();
        shareDirPath = shareDirString;

        char* home = getenv("HOME");
        if(home){
            homeDirString = home;
            homeDirPath = homeDirString;
        } else {
            homeDirString.clear();
            homeDirPath.clear();
        }
    } else {
        topDirString.clear();
        topDirPath.clear();
        shareDirString.clear();
        shareDirPath.clear();
        homeDirString.clear();
        homeDirPath.clear();
    }
}


void FilePathVariableProcessor::setUserVariables(Mapping* variables)
{
    impl->userVariables = variables;
}


void FilePathVariableProcessor::setBaseDirectory(const std::string& directory)
{
    impl->baseDirPath = filesystem::path(directory);
    if(impl->baseDirPath.is_relative()){
        impl->baseDirPath = filesystem::current_path() /  impl->baseDirPath;
    }

    if(impl->homeDirPath.empty()){
        impl->isBaseDirInHome = false;
    } else {
        filesystem::path relativePath;
        impl->isBaseDirInHome = findSubDirectory(impl->homeDirPath, impl->baseDirPath, relativePath);
    }
}


std::string FilePathVariableProcessor::baseDirectory() const
{
    return impl->baseDirPath.string();
}


void FilePathVariableProcessor::setProjectDirectory(const std::string& directory)
{
    impl->projectDirPath = directory;
    if(impl->projectDirPath.is_relative()){
        impl->projectDirPath = filesystem::current_path() /  impl->projectDirPath;
    }
    impl->projectDirString = impl->projectDirPath.generic_string();
}


const std::string& FilePathVariableProcessor::projectDirectory() const
{
    return impl->projectDirString;
}
    

std::string FilePathVariableProcessor::parameterize(const std::string& orgPathString)
{
    return impl->parameterize(orgPathString);
}


/**
   \todo Use integated nested map whose node is a single path element to be more efficient.
*/
std::string FilePathVariableProcessor::Impl::parameterize(const std::string& orgPathString)
{
    filesystem::path orgPath(orgPathString);

    // In the case where the path is originally relative one
    if(!orgPath.is_absolute()){
        return orgPath.generic_string();

    } else {
        filesystem::path relativePath;
        string varName;

        if(!baseDirPath.empty() && findSubDirectory(baseDirPath, orgPath, relativePath)){
            return relativePath.generic_string();
    
        } else if(findSubDirectory(projectDirPath, orgPath, relativePath)){
            return string("${PROJECT_DIR}/") + relativePath.generic_string();

        } else if(findSubDirectoryOfDirectoryVariable(orgPath, varName, relativePath)){
            return format("${{{0}}}/{1}", varName, relativePath.generic_string());

        } else if(isSystemVariableSetEnabled){

            if(findSubDirectory(shareDirPath, orgPath, relativePath)){
                return string("${SHARE}/") + relativePath.generic_string();

            } else if(findSubDirectory(topDirPath, orgPath, relativePath)){
                return string("${PROGRAM_TOP}/") + relativePath.generic_string();

            } else if(!isBaseDirInHome && findSubDirectory(homeDirPath, orgPath, relativePath)){
                return string("${HOME}/") + relativePath.generic_string();

            } else if(!baseDirPath.empty() && findRelativePath(baseDirPath, orgPath, relativePath)){
                return relativePath.generic_string();
            }
        }
    }
    
    return orgPath.generic_string();
}


/**
   \todo Introduce a tree structure to improve the efficiency of searching matched directories
*/
bool FilePathVariableProcessor::Impl::findSubDirectoryOfDirectoryVariable
(const filesystem::path& path, std::string& out_varName, filesystem::path& out_relativePath)
{
    out_relativePath.clear();
    int maxMatchSize = 0;
    filesystem::path relativePath;
    Mapping::const_iterator p;
    for(p = userVariables->begin(); p != userVariables->end(); ++p){
        Listing* paths = p->second->toListing();
        if(paths){
            for(int i=0; i < paths->size(); ++i){
                filesystem::path dirPath(paths->at(i)->toString());
                int n = findSubDirectory(dirPath, path, relativePath);
                if(n > maxMatchSize){
                    maxMatchSize = n;
                    out_relativePath = relativePath;
                    out_varName = fromUTF8(p->first);
                }
            }
        }
    }
    return (maxMatchSize > 0);
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

    string expanded = pathString;
    smatch match;

    if(regex_search(pathString, match, *variableRegex)){
        int pos = match.position();
        int len = match.length();
        string varname = match.str(1);

        if(isSystemVariableSetEnabled && varname == "SHARE"){
            expanded.replace(pos, len, shareDirString);
            
        } else if(isSystemVariableSetEnabled && varname == "PROGRAM_TOP"){
            expanded.replace(pos, len, topDirString);
            
        } else if(varname == "PROJECT_DIR"){
            if(projectDirString.empty()){
                errorMessage = format(_("${{PROJECT_DIR}} of \"{0}\" cannot be expanded."), pathString);
                expanded.clear();
            } else {
                expanded.replace(pos, len, projectDirString);
            }
            
        } else if(isSystemVariableSetEnabled && varname == "HOME"){
            expanded.replace(pos, len, homeDirString);
            
        } else {
            if(!replaceUserVariable(expanded, varname, pos, len)){
                expanded.clear();
            }
        }
    }

    if(!doMakeNativeAbsolutePath || expanded.empty()){
        return expanded;
    }
    
    filesystem::path path(expanded);
    if(!path.is_absolute()){
        if(!baseDirPath.empty()){
            path = baseDirPath / path;
        } else {
            path = filesystem::current_path() / path;
        }
    }
    path = filesystem::lexically_normal(path);
    return path.make_preferred().string(); // path.native() ?
}


bool FilePathVariableProcessor::Impl::replaceUserVariable
(string& io_pathString, const string& varname, int pos, int len)
{
    Listing* paths = userVariables->findListing(varname);

    if(!paths->isValid()){
        errorMessage = format(_("{0} of \"{1}\" is not defined."), varname, io_pathString);
        return false;
    }

    string replaced(io_pathString);
    replaced.replace(pos, len, paths->at(0)->toString());

    if(!filesystem::exists(replaced)){
        for(int i=1; i < paths->size(); ++i){
            string replaced2(io_pathString);
            replaced2.replace(pos, len, paths->at(i)->toString());
            if(filesystem::exists(replaced2)){
                replaced = std::move(replaced2);
                break;
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
