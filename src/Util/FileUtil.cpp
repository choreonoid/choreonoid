/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "FileUtil.h"

using namespace std;

namespace cnoid {

namespace filesystem = stdx::filesystem;

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
    return stdx::filesystem::lexically_normal(path);
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
    io_path = stdx::filesystem::lexically_normal(io_path);
    //io_path = getCompactPath(io_path);
}


/**
   \todo Replace this function with std::filesystem::path::lexically_relative
*/
int findSubDirectory(const filesystem::path& directory, const filesystem::path& path, filesystem::path& out_subdirectory)
{
    int numMatchedDepth = 0;
        
    if(directory.is_absolute() && path.is_absolute()){
        filesystem::path compactPath = filesystem::lexically_normal(path);

        filesystem::path::const_iterator p = directory.begin();
        filesystem::path::const_iterator q = compactPath.begin();

        while(p != directory.end() && q != compactPath.end()){
            if(!(*p == *q)){
                break;
            }
            ++numMatchedDepth;
            ++p;
            ++q;
        }
            
        if(p == directory.end()){
            out_subdirectory.clear();
            while(q != compactPath.end()){
                out_subdirectory /= *q++;
            }
            return numMatchedDepth;
        }
    }

    return 0;
}


bool findRelativePath(const filesystem::path& from_, const filesystem::path& to, filesystem::path& out_relativePath)
{
#if __cplusplus > 201402L
    out_relativePath = to.lexically_relative(from_);
    return !out_relativePath.empty();

#else
    if(from_.is_absolute() && to.is_absolute()){

        filesystem::path from(filesystem::lexically_normal(from_));
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


std::string getExtension(const stdx::filesystem::path& path)
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

std::string getGenericPathString(const stdx::filesystem::path& path)
{
    return path.generic_string();
}

bool checkAbsolute(const stdx::filesystem::path& path)
{
    return path.is_absolute();
}

stdx::filesystem::path getAbsolutePath(const stdx::filesystem::path& path)
{
    return stdx::filesystem::absolute(path);
}

std::string getAbsolutePathString(const stdx::filesystem::path& path)
{
    return stdx::filesystem::absolute(path).string();
}

std::string getFilename(const stdx::filesystem::path& path)
{
    return path.filename().string();
}

std::string getFilename(const std::string& pathString)
{
    stdx::filesystem::path path(pathString);
    return path.filename().string();
}

std::string getBasename(const stdx::filesystem::path& path)
{
    return path.stem().string();
}

std::string getPathString(const stdx::filesystem::path& path)
{
    return path.string();
}

std::string getNativePathString(const stdx::filesystem::path& path)
{
    stdx::filesystem::path p(path);
    return p.make_preferred().string();
}

std::string toActualPathName(const std::string& path)
{
    return path;
}

}
