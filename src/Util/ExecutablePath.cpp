/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ExecutablePath.h"
#include "FileUtil.h"

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __linux__
#include <sys/utsname.h>
#include <unistd.h>
#include <cstring>
#endif

#ifndef MACOSX
#if defined(__APPLE__) && defined(__MACH__)
#define	MACOSX
#endif
#endif

#ifdef MACOSX
#include <mach-o/dyld.h>
#endif

using namespace std;

namespace {
string executablePath_;
string executableDirectory_;
string executableTopDirectory_;
string pluginDirectory_;
string shareDirectory_;
string executableBasename_;
}


namespace cnoid {

namespace filesystem = stdx::filesystem;

void findExecutablePath()
{
#ifdef _WIN32
    static const int BUFSIZE = 1024;
    TCHAR execFilePath[BUFSIZE];
    if(GetModuleFileName(NULL, execFilePath, BUFSIZE)){
#ifndef UNICODE
        executablePath_ = execFilePath;
#else
        int codepage = _getmbcp();
        const int newSize = WideCharToMultiByte(codepage, 0, execFilePath, -1, NULL, 0, NULL, NULL);
        if(newSize > 0){
            vector<filesystem::path::String> execFilePathMB(newSize + 1);
            newSize = WideCharToMultiByte(codepage, 0, execFilePath, -1, &execFilePathMB[0], newSize + 1, NULL, NULL);
            executablePath_ = execFilePathUtf8;
            ;
        }
#endif // UNICODE
    }
#endif

#ifdef __linux__
    utsname info;
    if(uname(&info) == 0){
        if(strncmp(info.sysname, "Linux", 6) == 0){
            static const int BUFSIZE = 1024;
            char buf[BUFSIZE];
            int n = readlink("/proc/self/exe", buf, BUFSIZE - 1);
            buf[n] = 0;
            executablePath_ = buf;
        }
    }
#endif

#ifdef MACOSX
    char buf[1024];
    uint32_t n = sizeof(buf);
    if(_NSGetExecutablePath(buf, &n) == 0){
        executablePath_ = buf;
    }
        
    filesystem::path path;
    // remove dot from a path like bin/./choreonoid
    makePathCompact(filesystem::path(executablePath_), path);
    //filesystem::path path = filesystem::canonical(filesystem::path(executablePath_));
#else
    filesystem::path path(executablePath_);
#endif

    executableDirectory_ = path.parent_path().string();
    
    filesystem::path topPath = path.parent_path().parent_path();
    executableTopDirectory_ = topPath.string();

    filesystem::path pluginPath = topPath / CNOID_PLUGIN_SUBDIR;
    pluginDirectory_ = pluginPath.string();
        
    filesystem::path sharePath = topPath / CNOID_SHARE_SUBDIR;
    if(filesystem::is_directory(sharePath)){
        shareDirectory_ = getNativePathString(sharePath);

    } else if(filesystem::is_directory(sharePath.parent_path())){
        shareDirectory_ = getNativePathString(sharePath.parent_path());

    } else if(topPath.has_parent_path()){ // case of a sub build directory
        sharePath = topPath.parent_path() / "share";
        if(filesystem::is_directory(sharePath)){
            shareDirectory_ = getNativePathString(sharePath);
        }
    }

#ifdef _WIN32
    if(path.extension() == ".exe"){
        executableBasename_ = getBasename(path);
    } else {
        executableBasename_ = getFilename(path);
    }
#else
    executableBasename_ = getFilename(path);
#endif
}

const std::string& executablePath()
{
    if(executablePath_.empty()){
        findExecutablePath();
    }
    return executablePath_;
}

const std::string& executableDirectory()
{
    if(executablePath_.empty()){
        findExecutablePath();
    }
    return executableDirectory_;
}

const std::string& executableTopDirectory()
{
    if(executablePath_.empty()){
        findExecutablePath();
    }
    return executableTopDirectory_;
}

const std::string& pluginDirectory()
{
    if(executablePath_.empty()){
        findExecutablePath();
    }
    return pluginDirectory_;
}

const std::string& shareDirectory()
{
    if(executablePath_.empty()){
        findExecutablePath();
    }
    return shareDirectory_;
}

const std::string& executableBasename()
{
    if(executablePath_.empty()){
        findExecutablePath();
    }
    return executableBasename_;
}

}
