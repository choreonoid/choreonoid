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
    filesystem::path compact;
    
    for(filesystem::path::const_iterator p = path.begin(); p != path.end(); ++p){
        if(*p == ".."){
            compact = compact.parent_path();
        } else if(*p != "."){
            compact /= *p;
        }
    }

    return compact;
}


void makePathCompact(filesystem::path& io_path)
{
    io_path = getCompactPath(io_path);
}


int findSubDirectory(const filesystem::path& directory, const filesystem::path& path, filesystem::path& out_subdirectory)
{
    int numMatchedDepth = 0;
        
    if(directory.is_absolute() && path.is_absolute()){
        filesystem::path compactPath = getCompactPath(path);

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
    if(from_.is_absolute() && to.is_absolute()){

        filesystem::path from(getCompactPath(from_));
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
}


#ifndef CNOID_FILE_UTIL_SUPPORT_WINDOWS_FILESYSTEM
std::string toActualPathName(const std::string& path)
{
    return path;
}

#else
/**
   \note This implementation cannot be used because the extension is omitted
   when the exploer hides the extension of registered file types.
   \todo Support the case where the exploer hides the extension of registered file types.
*/
std::string toActualPathName(const std::string& path)
{
    int codepage = _getmbcp();
    size_t length = ::MultiByteToWideChar(codepage, 0, path.c_str(), path.size(), NULL, 0);
    if(length >= 0){
        wchar_t wpath[MAX_PATH];
        ::MultiByteToWideChar(codepage, 0, path.c_str(), path.size(), wpath, MAX_PATH);
        
        // The following code was based on the code posted at
        // http://stackoverflow.com/questions/74451/getting-actual-file-name-with-proper-casing-on-windowsis
        // Thank you.
        
        const wchar_t kSeparator = L'\\';
        size_t i = 0;
        std::wstring result;
        
        // for network paths (\\server\share\RestOfPath), getting the display
        // name mangles it into unusable form (e.g. "\\server\share" turns
        // into "share on server (server)"). So detect this case and just skip
        // up to two path components
        if( length >= 2 && wpath[0] == kSeparator && wpath[1] == kSeparator ) {
            int skippedCount = 0;
            i = 2; // start after '\\'
            while( i < length && skippedCount < 2 ) {
                if( wpath[i] == kSeparator ){
                    ++skippedCount;
                }
                ++i;
            }
            result.append( wpath, i );
        }
        // for drive names, just add it uppercased
        else if( length >= 2 && wpath[1] == L':' ) {
            result += towupper(wpath[0]);
            result += L':';
            if( length >= 3 && wpath[2] == kSeparator ){
                result += kSeparator;
                i = 3; // start after drive, colon and separator
            } else {
                i = 2; // start after drive and colon
            }
        }
        
        size_t lastComponentStart = i;
        bool addSeparator = false;
        
        while( i < length ) {
            // skip until path separator
            while( i < length && wpath[i] != kSeparator ) {
                ++i;
            }
            if( addSeparator ) {
                result += kSeparator;
            }
            
            // if we found path separator, get real filename of this
            // last path name component
            bool foundSeparator = (i < length);
            wpath[i] = 0;
            SHFILEINFOW info;
            
            // nuke the path separator so that we get real name of current path component
            info.szDisplayName[0] = 0;
            if( SHGetFileInfoW( wpath, 0, &info, sizeof(info), SHGFI_DISPLAYNAME ) ) {
                result += info.szDisplayName;
            } else {
                // most likely file does not exist.
                // So just append original path name component.
                result.append( wpath + lastComponentStart, i - lastComponentStart );
            }
            
            // restore path separator that we might have nuked before
            if( foundSeparator ){
                wpath[i] = kSeparator;
            }
            
            ++i;
            lastComponentStart = i;
            addSeparator = true;
        }
        
        length = ::WideCharToMultiByte(codepage, 0,  &result[0], result.size(), NULL, 0, NULL, NULL);
        if(length >= 0){
            std::vector<char> converted(length + 1);
            ::WideCharToMultiByte(codepage, 0,  &result[0], result.size(), &converted[0], length + 1, NULL, NULL);
            return std::string(&converted[0], length);
        }
    }
    return path; // failed
}
#endif


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

}
