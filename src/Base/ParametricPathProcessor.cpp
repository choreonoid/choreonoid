/**
   @author Shin'ichiro Nakaoka
*/

#include "ParametricPathProcessor.h"
#include <cnoid/ValueTree>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/UTF8>
#include <QRegExp>
#include <boost/format.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;
namespace filesystem = boost::filesystem;

namespace {

QRegExp regexVar("^\\$\\{(\\w+)\\}");

}

namespace cnoid {

class ParametricPathProcessorImpl
{
public:
    MappingPtr variables;
    QString topDirString;
    QString shareDirString;
    QString homeDirString;
    filesystem::path baseDirPath;
    filesystem::path topDirPath;
    filesystem::path shareDirPath;
    filesystem::path homeDirPath;
    bool isBaseDirInHome;
    string errorMessage;

    ParametricPathProcessorImpl();
    bool findSubDirectoryOfDirectoryVariable(
        const filesystem::path& path, std::string& out_varName, filesystem::path& out_relativePath);
    bool replaceDirectoryVariable(QString& io_pathString, const string& varname, int pos, int len);
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
    isBaseDirInHome = false;
    
    const std::string& top = executableTopDirectory();
    topDirPath = top;
    topDirString = top.c_str();

    const std::string& share = shareDirectory();
    shareDirPath = share;
    shareDirString = share.c_str();

    char* home = getenv("HOME");
    if(home){
        homeDirPath = filesystem::path(home);
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
    impl->baseDirPath = directory;
    filesystem::path relativePath;
    impl->isBaseDirInHome = findSubDirectory(impl->homeDirPath, impl->baseDirPath, relativePath);
}
    

/**
   \todo Use integated nested map whose node is a single path element to be more efficient.
*/
std::string ParametricPathProcessor::parameterize(const std::string& orgPathString) const
{
    filesystem::path orgPath(orgPathString);

    // In the case where the path is originally relative one
    if(!orgPath.is_complete()){
        return getGenericPathString(orgPath);

    } else {
        filesystem::path relativePath;

        if(!impl->baseDirPath.empty() && findSubDirectory(impl->baseDirPath, orgPath, relativePath)){
            return getGenericPathString(relativePath);
    
        } else {
            string varName;
            if(impl->findSubDirectoryOfDirectoryVariable(orgPath, varName, relativePath)){
                return str(format("${%1%}/%2%") % varName % getGenericPathString(relativePath));

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
(const filesystem::path& path, std::string& out_varName, filesystem::path& out_relativePath)
{
    out_relativePath.clear();
    int maxMatchSize = 0;
    filesystem::path relativePath;
    Mapping::const_iterator p;
    for(p = variables->begin(); p != variables->end(); ++p){
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


boost::optional<std::string> ParametricPathProcessor::expand(const std::string& pathString) const
{
    filesystem::path path;
    
    QString qpath(pathString.c_str());

    // expand variables in the path
    int pos = regexVar.indexIn(qpath);
    if(pos == -1){
        path = pathString;
    } else {
        int len = regexVar.matchedLength();
        if(regexVar.captureCount() > 0){
            QString varname = regexVar.cap(1);
            if(varname == "SHARE"){
                qpath.replace(pos, len, impl->shareDirString);
            } else if(varname == "PROGRAM_TOP"){
                qpath.replace(pos, len, impl->topDirString);
            } else if(varname == "HOME"){
                qpath.replace(pos, len, impl->homeDirString);
            } else {
                if(!impl->replaceDirectoryVariable(qpath, varname.toStdString(), pos, len)){
                    return boost::none;
                }
            }
        }
        path = qpath.toStdString();
    }
            
    if(checkAbsolute(path) || impl->baseDirPath.empty()){
        return getNativePathString(path);
    } else {
        filesystem::path fullPath = impl->baseDirPath / path;
        if(!path.empty() && (*path.begin() == "..")){
            filesystem::path compact(getCompactPath(fullPath));
            return getNativePathString(compact);
        } else {
            return getNativePathString(fullPath);
        }
    }
}


bool ParametricPathProcessorImpl::replaceDirectoryVariable
(QString& io_pathString, const string& varname, int pos, int len)
{
    Listing* paths = variables->findListing(varname);
    if(paths){
        for(int i=0; i < paths->size(); ++i){
            string vpath;
            QString replaced(io_pathString);
            replaced.replace(pos, len, paths->at(i)->toString().c_str());
            filesystem::file_status fstatus = filesystem::status(filesystem::path(replaced.toStdString()));
            if(filesystem::is_directory(fstatus) || filesystem::exists(fstatus)) {
                io_pathString = replaced;
                return true;
            }
        }
    }

    errorMessage = str(format(_("%1% of \"%2%\" cannot be expanded !")) % varname % io_pathString.toStdString());
    return false;
}


const std::string& ParametricPathProcessor::errorMessage() const
{
    return impl->errorMessage;
}

