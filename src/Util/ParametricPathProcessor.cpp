/**
   @author Shin'ichiro Nakaoka
*/

#include "ParametricPathProcessor.h"
#include "ValueTree.h"
#include "ExecutablePath.h"
#include "FileUtil.h"
#include "UTF8.h"
#include <fmt/format.h>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class ParametricPathProcessorImpl
{
public:
    MappingPtr variables;
    regex variableRegex;
    bool isVariableRegexAssinged;
    string baseDirString;
    stdx::filesystem::path baseDirPath;
    bool isBaseDirInHome;
    string topDirString;
    stdx::filesystem::path topDirPath;
    string shareDirString;
    stdx::filesystem::path shareDirPath;
    string homeDirString;
    stdx::filesystem::path homeDirPath;
    string projectDirString;
    string errorMessage;

    ParametricPathProcessorImpl();
    bool findSubDirectoryOfDirectoryVariable(
        const stdx::filesystem::path& path, std::string& out_varName, stdx::filesystem::path& out_relativePath);
    bool replaceDirectoryVariable(string& io_pathString, const string& varname, int pos, int len);
    stdx::optional<std::string> expand(const std::string& pathString);
};

}


ParametricPathProcessor* ParametricPathProcessor::instance()
{
    static ParametricPathProcessor* instance_ = new ParametricPathProcessor();
    return instance_;
}


ParametricPathProcessor::ParametricPathProcessor()
{
    impl = new ParametricPathProcessorImpl();
}


ParametricPathProcessorImpl::ParametricPathProcessorImpl()
{
    isVariableRegexAssinged = false;
    isBaseDirInHome = false;
    
    const std::string& top = executableTopDirectory();
    topDirPath = top;
    topDirString = top.c_str();

    const std::string& share = shareDirectory();
    shareDirPath = share;
    shareDirString = share.c_str();

    char* home = getenv("HOME");
    if(home){
        homeDirPath = stdx::filesystem::path(home);
        homeDirString = home;
    }
}


ParametricPathProcessor::~ParametricPathProcessor()
{
    delete impl;
}


void ParametricPathProcessor::setVariables(Mapping* variables)
{
    impl->variables = variables;
}


void ParametricPathProcessor::setBaseDirectory(const std::string& directory)
{
    impl->baseDirString = directory;
    impl->baseDirPath = directory;
    stdx::filesystem::path relativePath;
    impl->isBaseDirInHome = findSubDirectory(impl->homeDirPath, impl->baseDirPath, relativePath);
}


void ParametricPathProcessor::setProjectDirectory(const std::string& directory)
{
    impl->projectDirString = directory;
}
    

/**
   \todo Use integated nested map whose node is a single path element to be more efficient.
*/
std::string ParametricPathProcessor::parameterize(const std::string& orgPathString)
{
    stdx::filesystem::path orgPath(orgPathString);

    // In the case where the path is originally relative one
    if(!orgPath.is_absolute()){
        return getGenericPathString(orgPath);

    } else {
        stdx::filesystem::path relativePath;

        if(!impl->baseDirPath.empty() && findSubDirectory(impl->baseDirPath, orgPath, relativePath)){
            return getGenericPathString(relativePath);
    
        } else {
            string varName;
            if(impl->findSubDirectoryOfDirectoryVariable(orgPath, varName, relativePath)){
                return format("${{{0}}}/{1}", varName, getGenericPathString(relativePath));

            } else if(findSubDirectory(impl->shareDirPath, orgPath, relativePath)){
                return string("${SHARE}/") + getGenericPathString(relativePath);

            } else if(findSubDirectory(impl->topDirPath, orgPath, relativePath)){
                return string("${PROGRAM_TOP}/") + getGenericPathString(relativePath);

            } else if(!impl->isBaseDirInHome && findSubDirectory(impl->homeDirPath, orgPath, relativePath)){
                return string("${HOME}/") + getGenericPathString(relativePath);

            } else if(!impl->baseDirPath.empty() && findRelativePath(impl->baseDirPath, orgPath, relativePath)){
                return getGenericPathString(relativePath);
            }
        }
    }
    return getGenericPathString(orgPath);
}


/**
   \todo Introduce a tree structure to improve the efficiency of searching matched directories
*/
bool ParametricPathProcessorImpl::findSubDirectoryOfDirectoryVariable
(const stdx::filesystem::path& path, std::string& out_varName, stdx::filesystem::path& out_relativePath)
{
    out_relativePath.clear();
    int maxMatchSize = 0;
    stdx::filesystem::path relativePath;
    Mapping::const_iterator p;
    for(p = variables->begin(); p != variables->end(); ++p){
        Listing* paths = p->second->toListing();
        if(paths){
            for(int i=0; i < paths->size(); ++i){
                stdx::filesystem::path dirPath(paths->at(i)->toString());
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


stdx::optional<std::string> ParametricPathProcessor::expand(const std::string& pathString)
{
    return impl->expand(pathString);
}


stdx::optional<std::string> ParametricPathProcessorImpl::expand(const std::string& pathString)
{
    if(!isVariableRegexAssinged){
        variableRegex = "^\\$\\{(\\w+)\\}";
        isVariableRegexAssinged = true;
    }

    stdx::filesystem::path path;
    smatch match;
    if(!regex_search(pathString, match, variableRegex)){
        path = pathString;
    } else {
        int pos = match.position();
        int len = match.length();
        string varname = match.str(1);
        string expanded(pathString);
        if(varname == "SHARE"){
            expanded.replace(pos, len, shareDirString);
        } else if(varname == "PROGRAM_TOP"){
            expanded.replace(pos, len, topDirString);
        } else if(varname == "HOME"){
            expanded.replace(pos, len, homeDirString);
        } else if(varname == "PROJECT_DIR"){
            if(projectDirString.empty()){
                errorMessage =
                    format(_("PROJECT_DIR of \"{}\" cannot be expanded."), pathString);
                return stdx::nullopt;
            }
            expanded.replace(pos, len, projectDirString);
        } else {
            if(!replaceDirectoryVariable(expanded, varname, pos, len)){
                return stdx::nullopt;
            }
        }
        path = expanded;
    }

    if(checkAbsolute(path) || baseDirPath.empty()){
        return getNativePathString(path);
    } else {
        stdx::filesystem::path fullPath = baseDirPath / path;
        if(!path.empty() && (*path.begin() == "..")){
            stdx::filesystem::path compact(getCompactPath(fullPath));
            return getNativePathString(compact);
        } else {
            return getNativePathString(fullPath);
        }
    }
}


bool ParametricPathProcessorImpl::replaceDirectoryVariable
(string& io_pathString, const string& varname, int pos, int len)
{
    Listing* paths = variables->findListing(varname);
    if(paths){
        for(int i=0; i < paths->size(); ++i){
            string vpath;
            string replaced(io_pathString);
            replaced.replace(pos, len, paths->at(i)->toString());
            stdx::filesystem::file_status fstatus = stdx::filesystem::status(stdx::filesystem::path(replaced));
            if(stdx::filesystem::is_directory(fstatus) || stdx::filesystem::exists(fstatus)) {
                io_pathString = replaced;
                return true;
            }
        }
    }

    errorMessage = format(_("{0} of \"{1}\" cannot be expanded."), varname, io_pathString);
    return false;
}


const std::string& ParametricPathProcessor::errorMessage() const
{
    return impl->errorMessage;
}

