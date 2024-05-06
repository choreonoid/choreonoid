#include <cnoid/UriSchemeProcessor>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <vector>
#include <cstdlib>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = stdx::filesystem;

namespace {

/*
  This handler enables BodyLoader classes to support the "package://" scheme used in ROS.
*/
class ROSPackageSchemeHandler
{
    vector<string> packagePaths;
    
public:
    ROSPackageSchemeHandler()
    {
        const char* rpp = getenv("ROS_PACKAGE_PATH"); // for ROS 1
        if(rpp){
            do {
                const char* begin = rpp;
                while(*rpp != ':' && *rpp) rpp++;
                packagePaths.push_back(string(begin, rpp));
            } while (0 != *rpp++);
        }
        
        const char* app = getenv("AMENT_PREFIX_PATH"); // for ROS 2
        if(app){
            do {
                const char* begin = app;
                while(*app != ':' && *app) app++;
                filesystem::path path(string(begin, app));
                path /= "share";
                packagePaths.push_back(path.string());
            } while (0 != *app++);
        }
    }

    string operator()(const string& path, std::ostream& os)
    {
        filesystem::path filepath(fromUTF8(path));
        auto iter = filepath.begin();
        if(iter == filepath.end()){
            return string();
        }
        
        filesystem::path directory = *iter++;
        filesystem::path relativePath;
        while(iter != filepath.end()){
            relativePath /= *iter++;
        }

        bool found = false;
        filesystem::path combined;
        
        for(auto element : packagePaths){
            filesystem::path packagePath(element);
            combined = packagePath / filepath;
            if(exists(combined)){
                found = true;
                break;
            }
            combined = packagePath / relativePath;
            if(exists(combined)){
                found = true;
                break;
            }
        }

        if(found){
            return toUTF8(combined.string());
        } else {
            os << fmt::format(_("\"{}\" is not found in the ROS package directories."), path);
            os.flush();
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
