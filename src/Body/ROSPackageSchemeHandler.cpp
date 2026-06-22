#include <cnoid/UriSchemeProcessor>
#include <cnoid/UTF8>
#include <cnoid/Format>
#include <filesystem>
#include <vector>
#include <cstdlib>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

// Separator of the ROS_PACKAGE_PATH / AMENT_PREFIX_PATH path lists. Windows uses ';' because
// its absolute paths contain ':' in drive letters such as "C:\...".
#ifdef _WIN32
constexpr char pathListSeparator = ';';
#else
constexpr char pathListSeparator = ':';
#endif

// Checks both search-path styles used by this handler: a directory containing the package
// (".../share" or a ROS_PACKAGE_PATH entry) and the package directory itself.
bool findFileInPackageDirectory(
    const filesystem::path& directory, const filesystem::path& filepath,
    const filesystem::path& relativePath, bool enableRelativePath,
    filesystem::path& out_filepath)
{
    filesystem::path combined = directory / filepath;
    if (exists(combined)) {
        out_filepath = combined;
        return true;
    }
    if (enableRelativePath && !relativePath.empty()) {
        combined = directory / relativePath;
        if (exists(combined)) {
            out_filepath = combined;
            return true;
        }
    }
    return false;
}

/*
  This handler enables BodyLoader classes to support the "package://" scheme used in ROS.
*/
class ROSPackageSchemeHandler
{
    vector<string> packagePaths;
    bool has_ROS_PACKAGE_PATH;
    
public:
    ROSPackageSchemeHandler()
    {
        has_ROS_PACKAGE_PATH = false;
        const char* rpp = getenv("ROS_PACKAGE_PATH"); // for ROS 1
        if(rpp){
            do {
                const char* begin = rpp;
                while(*rpp != pathListSeparator && *rpp) rpp++;
                string element(begin, rpp);
                if(!element.empty()){
                    packagePaths.push_back(element);
                    has_ROS_PACKAGE_PATH = true;
                }
            } while (0 != *rpp++);
        }
        
        const char* app = getenv("AMENT_PREFIX_PATH"); // for ROS 2
        if(app){
            do {
                const char* begin = app;
                while(*app != pathListSeparator && *app) app++;
                string element(begin, app);
                if(!element.empty()){
                    filesystem::path path(element);
                    path /= "share";
                    packagePaths.push_back(path.string());
                }
            } while (0 != *app++);
        }
    }

    string operator()(const string& path, UriSchemeProcessor& processor)
    {
        filesystem::path filepath(fromUTF8(path));
        auto iter = filepath.begin();
        if(iter == filepath.end()){
            return string();
        }

        filesystem::path relativePath;
        ++iter;
        while(iter != filepath.end()){
            relativePath /= *iter++;
        }

        bool found = false;
        filesystem::path combined;

        // Per-loader local search paths are inferred from the file being loaded. Try them
        // first so self-contained model packages override similarly named packages elsewhere.
        for(auto element : processor.modelSearchDirectories()){
            filesystem::path packagePath(element);
            if(findFileInPackageDirectory(packagePath, filepath, relativePath, true, combined)){
                found = true;
                break;
            }
        }

        // Environment-derived paths come next. These are collected from ROS_PACKAGE_PATH and
        // AMENT_PREFIX_PATH when this package URI handler is constructed.
        for(auto element : packagePaths){
            if(found){
                break;
            }
            filesystem::path packagePath(element);
            found = findFileInPackageDirectory(
                packagePath, filepath, relativePath, has_ROS_PACKAGE_PATH, combined);
        }

        if(found){
            return toUTF8(combined.string());
        } else {
            processor.setErrorMessage(
                formatR(_("\"{}\" is not found in the ROS package directories."), path));
            return string();
        }
    }
};

struct ROSPackageSchemeHandlerRegistration {
    ROSPackageSchemeHandlerRegistration(){
        UriSchemeProcessor::registerUriSchemeHandler("package", ROSPackageSchemeHandler());
    }
} rosPackageSchemeHandlerRegistration;

}
