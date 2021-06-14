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
    filesystem::path projectDirPath;
    string projectDirString;
    bool isProjectDirDifferentFromBaseDir;

    bool isSystemVariableSetEnabled;
    filesystem::path topDirPath;
    string topDirString;
    filesystem::path shareDirPath;
    string shareDirString;
    filesystem::path homeDirPath;
    string homeDirString;

    string varName;
    enum BaseType { Base, Project, Var, Share, Top, Home, NumCandidates };
    filesystem::path relativePath[NumCandidates];

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
    isProjectDirDifferentFromBaseDir = false;
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
        topDirString = executableTopDir();
        topDirPath = executableTopDirPath();

        shareDirString = shareDir();
        shareDirPath = cnoid::shareDirPath();

        char* home = getenv("HOME");
        if(home){
            homeDirString = toUTF8(home);
            homeDirPath = string(home);
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
    impl->baseDirPath = filesystem::path(fromUTF8(directory));
    if(impl->baseDirPath.is_relative()){
        impl->baseDirPath = filesystem::current_path() /  impl->baseDirPath;
    }
    impl->isProjectDirDifferentFromBaseDir = (impl->baseDirPath != impl->projectDirPath);
}


void FilePathVariableProcessor::clearBaseDirectory()
{
    setBaseDirectory("");
}


std::string FilePathVariableProcessor::baseDirectory() const
{
    return toUTF8(impl->baseDirPath.string());
}


stdx::filesystem::path FilePathVariableProcessor::baseDirPath() const
{
    return impl->baseDirPath;
}


void FilePathVariableProcessor::setProjectDirectory(const std::string& directory)
{
    impl->projectDirPath = fromUTF8(directory);
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


std::string FilePathVariableProcessor::parameterize(const std::string& orgPathString)
{
    return impl->parameterize(orgPathString);
}


static int countPathElements(filesystem::path& path)
{
    int count = 0;
    bool hasUpElement = false;
    auto iter = path.begin();
    while(iter != path.end()){
        if(*iter == ".."){
            hasUpElement = true;
        }
        ++count;
        ++iter;
    }
    if(hasUpElement){
        ++count;
    }
    return count;
}


/**
   \todo Use integated nested map whose node is a single path element to be more efficient.
*/
std::string FilePathVariableProcessor::Impl::parameterize(const std::string& orgPathString)
{
    filesystem::path orgPath(fromUTF8(orgPathString));

    // In the case where the path is originally relative one
    if(!orgPath.is_absolute()){
        return toUTF8(orgPath.generic_string());

    } else {
        int distance[NumCandidates];
        for(int i=0; i < NumCandidates; ++i){
            distance[i] = std::numeric_limits<int>::max();
        }
        
        if(!baseDirPath.empty() &&
           findRelativePath(baseDirPath, orgPath, relativePath[Base])){
            distance[Base] = countPathElements(relativePath[Base]);
        }
        if(isProjectDirDifferentFromBaseDir &&
           findSubDirectory(projectDirPath, orgPath, relativePath[Project])){
            distance[Project] = countPathElements(relativePath[Project]);
        }
        if(userVariables && findSubDirectoryOfDirectoryVariable(orgPath, varName, relativePath[Var])){
            distance[Var] = countPathElements(relativePath[Var]);
        }
        if(isSystemVariableSetEnabled){

            if(findSubDirectory(shareDirPath, orgPath, relativePath[Share])){
                distance[Share] = countPathElements(relativePath[Share]);

            } else if(findSubDirectory(topDirPath, orgPath, relativePath[Top])){
                distance[Top] = countPathElements(relativePath[Top]);
            }
            if(findSubDirectory(homeDirPath, orgPath, relativePath[Home])){
                distance[Home] = countPathElements(relativePath[Home]) + 1;
            }
        }

        int minDistance = std::numeric_limits<int>::max();
        int index = -1;
        for(int i=0; i < NumCandidates; ++i){
            if(distance[i] < minDistance){
                minDistance = distance[i];
                index = i;
            }
        }
        if(index >= 0){
            switch(index){
            case Base:
                if(countPathElements(orgPath) < minDistance){
                    return toUTF8(orgPath.generic_string()); // absolute path
                } else {
                    return toUTF8(relativePath[Base].generic_string());
                }
            case Project:
                return string("${PROJECT_DIR}/") + toUTF8(relativePath[Project].generic_string());
            case Var:
                return format("${{{0}}}/{1}", varName, toUTF8(relativePath[Var].generic_string()));
            case Share:
                return string("${SHARE}/") + toUTF8(relativePath[Share].generic_string());
            case Top:
                return string("${PROGRAM_TOP}/") + toUTF8(relativePath[Top].generic_string());
            case Home:
                return string("${HOME}/") + toUTF8(relativePath[Home].generic_string());
            default:
                break;
            }
        }
    }
    
    return toUTF8(orgPath.generic_string());
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
                    out_varName = p->first;
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

    bool expansionFailed = false;
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
                expansionFailed = true;
                expanded.clear();
            } else {
                expanded.replace(pos, len, projectDirString);
            }
            
        } else if(isSystemVariableSetEnabled && varname == "HOME"){
            expanded.replace(pos, len, homeDirString);
            
        } else {
            if(!replaceUserVariable(expanded, varname, pos, len)){
                expansionFailed = true;
                expanded.clear();
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
(string& io_pathString, const string& varname, int pos, int len)
{
    Listing* paths = nullptr;
    if(userVariables){
        paths = userVariables->findListing(varname);
    }
    if(!paths || !paths->isValid()){
        errorMessage = format(_("The \"{0}\" variable in \"{1}\" is not defined."), varname, io_pathString);
        return false;
    }

    string replaced(io_pathString);
    replaced.replace(pos, len, paths->at(0)->toString());

    if(!filesystem::exists(fromUTF8(replaced))){
        for(int i=1; i < paths->size(); ++i){
            string replaced2(io_pathString);
            replaced2.replace(pos, len, paths->at(i)->toString());
            if(filesystem::exists(fromUTF8(replaced2))){
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
