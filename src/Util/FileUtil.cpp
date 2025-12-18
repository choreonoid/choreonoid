#include "FileUtil.h"
#include <algorithm>

using namespace std;

namespace cnoid {


#ifdef _WIN32
const char* DLL_PREFIX = "";
const char* DLL_SUFFIX = ".dll";
const char* DLL_EXTENSION = "dll";
const char* EXEC_SUFFIX = ".exe";
const char* EXEC_EXTENSION = "exe";
const char* PATH_DELIMITER = ";";
#elif defined(__APPLE__)
const char* DLL_PREFIX = "lib";
const char* DLL_SUFFIX = ".dylib";
const char* DLL_EXTENSION = "dylib";
const char* EXEC_SUFFIX = ".app";
const char* EXEC_EXTENSION = "app";
const char* PATH_DELIMITER = ":";
#else
const char* DLL_PREFIX = "lib";
const char* DLL_SUFFIX = ".so";
const char* DLL_EXTENSION = "so";
const char* EXEC_SUFFIX = "";
const char* EXEC_EXTENSION = "";
const char* PATH_DELIMITER = ":";
#endif


filesystem::path getCompactPath(const filesystem::path& path)
{
    return path.lexically_normal();
    /*
    filesystem::path compact;
    for(filesystem::path::const_iterator p = path.begin(); p != path.end(); ++p){
        if(*p == ".."){
            compact = compact.parent_path();
        } else if(*p != "."){
            compact /= *p;
        }
    }
    return compact;
    */
}


void makePathCompact(filesystem::path& io_path)
{
    io_path = io_path.lexically_normal();
    //io_path = getCompactPath(io_path);
}


int findPathInDirectory(const filesystem::path& directory, const filesystem::path& path, filesystem::path& out_relativePath)
{
    int numMatchedDepth = 0;
        
    if(directory.is_absolute() && path.is_absolute()){
        filesystem::path compactPath = path.lexically_normal();

        filesystem::path::const_iterator it1 = directory.begin();
        filesystem::path::const_iterator it2 = compactPath.begin();

        while(it1 != directory.end() && it2 != compactPath.end()){
            bool matched = false;
#ifdef _WIN32
            auto p1 = *it1;
            auto s1 = p1.make_preferred().string();
            std::transform(s1.begin(), s1.end(), s1.begin(), ::tolower);
            auto p2 = *it2;
            auto s2 = p2.make_preferred().string();
            std::transform(s2.begin(), s2.end(), s2.begin(), ::tolower);

            // In Windows, directory and file names cannot end with a period(.).
            // Any trailing period should be removed for comarison purpose.
            if(!s1.empty() && s1.back() == '.'){
                s1.pop_back();
            }
            if(!s2.empty() && s2.back() == '.'){
                s2.pop_back();
            }
            
            if(s1 == s2){
                matched = true;
            }
#else
            if(*it1 == *it2){
                matched = true;
            }
#endif
            if(!matched){
                break;
            }
            ++numMatchedDepth;
            ++it1;
            ++it2;
        }
            
        if(it1 == directory.end()){
            out_relativePath.clear();
            while(it2 != compactPath.end()){
                out_relativePath /= *it2++;
            }
            return numMatchedDepth;
        }
    }

    return 0;
}


int findSubDirectory(const filesystem::path& directory, const filesystem::path& path, filesystem::path& out_subdirectory)
{
    return findPathInDirectory(directory, path, out_subdirectory);
}


std::optional<std::filesystem::path> getRelativePath(const filesystem::path& path_, const filesystem::path& base_)
{
    filesystem::path relativePath;
    bool isAbsolute;
    
    if(path_.is_absolute()){
        if(base_.is_absolute()){
            isAbsolute = true;
        } else {
            return std::nullopt;
        }
    } else {
        if(base_.is_absolute()){
            return std::nullopt;
        }
        isAbsolute = false;
    }

    filesystem::path path = filesystem::path(path_).lexically_normal();
    filesystem::path base = filesystem::path(base_).lexically_normal();
    auto it1 = path.begin();
    auto it2 = base.begin();
    bool isFirstElement = true;
        
    while(it1 != path.end() && it2 != base.end()){
        bool matched = false;
#ifdef _WIN32
        auto p1 = *it1;
        auto s1 = p1.make_preferred().string();
        std::transform(s1.begin(), s1.end(), s1.begin(), ::tolower);
        auto p2 = *it2;
        auto s2 = p2.make_preferred().string();
        std::transform(s2.begin(), s2.end(), s2.begin(), ::tolower);

        // In Windows, directory and file names cannot end with a period(.).
        // Any trailing period should be removed for comarison purpose.
        if(!s1.empty() && s1.back() == '.'){
            s1.pop_back();
        }
        if(!s2.empty() && s2.back() == '.'){
            s2.pop_back();
        }

        if(s1 == s2){
            matched = true;
        }
#else
        if(*it1 == *it2){
            matched = true;
        }
#endif
        if(!matched){
            break;
        }
        ++it1;
        ++it2;
        isFirstElement = false;
    }

    if(isFirstElement && isAbsolute){
        // There is no relative path
        return std::nullopt;
    }

    while(it2 != base.end()){
        relativePath /= "..";
        ++it2;
    }
    while(it1 != path.end()){
        relativePath /= *it1++;
    }

    return relativePath;
}


bool checkIfSubFilePath(const std::filesystem::path& path, const std::filesystem::path& base)
{
    bool result = false;
    if(auto relPath = getRelativePath(path, base)){
        result = true;
        for(auto& element : *relPath){
            if(element.string() == ".."){
                result = false;
                break;
            }
        }
    }
    return result;
}


bool findRelativePath(const filesystem::path& from_, const filesystem::path& to, filesystem::path& out_relativePath)
{
#if __cplusplus > 201402L && !defined(__cpp_lib_experimental_filesystem)
    out_relativePath = to.lexically_relative(from_);
    return !out_relativePath.empty();

#else
    if(from_.is_absolute() && to.is_absolute()){

        filesystem::path from = from_.lexically_normal();
        filesystem::path::const_iterator p = from.begin();
        filesystem::path::const_iterator q = to.begin();
        
        while(p != from.end() && q != to.end()){
            if(!(*p == *q)){
                break;
            }
            ++p;
            ++q;
        }

        out_relativePath.clear();

        while(p != from.end()){
            out_relativePath /= "..";
            ++p;
        }
        while(q != to.end()){
            out_relativePath /= *q++;
        }
        return true;
    }
    
    return false;
#endif
}


std::filesystem::path getNativeUniformPath(const std::filesystem::path& path)
{
    auto uniformed = path.lexically_normal();
    uniformed.make_preferred();

#ifndef _WIN32
    return uniformed;
#else
    filesystem::path lowercased;
    for(auto& element : uniformed){
        auto s = element.string();
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        lowercased /= s;
    }
    return lowercased;
#endif
}


std::string getExtension(const std::filesystem::path& path)
{
    string ext = path.extension().string();
    if(!ext.empty()){
        if(ext[0] == '.'){
            ext = ext.substr(1);
        } else {
            ext.clear();
        }
    }
    return ext;
}

std::string getGenericPathString(const std::filesystem::path& path)
{
    return path.generic_string();
}

bool checkAbsolute(const std::filesystem::path& path)
{
    return path.is_absolute();
}

std::filesystem::path getAbsolutePath(const std::filesystem::path& path)
{
    return std::filesystem::absolute(path);
}

std::string getAbsolutePathString(const std::filesystem::path& path)
{
    return std::filesystem::absolute(path).string();
}

std::string getFilename(const std::filesystem::path& path)
{
    return path.filename().string();
}

std::string getFilename(const std::string& pathString)
{
    std::filesystem::path path(pathString);
    return path.filename().string();
}

std::string getBasename(const std::filesystem::path& path)
{
    return path.stem().string();
}

std::string getPathString(const std::filesystem::path& path)
{
    return path.string();
}

std::string getNativePathString(const std::filesystem::path& path)
{
    std::filesystem::path p(path);
    return p.make_preferred().string();
}

std::string toActualPathName(const std::string& path)
{
    return path;
}

}
