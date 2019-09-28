/**
   @author Shin'ichiro Nakaoka
*/

#include "AppConfig.h"
#include "MessageView.h"
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/FileUtil>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;

namespace {

string application;
string organization;

filesystem::path configDirPath;
filesystem::path filePath;
filesystem::path fullPath;

std::shared_ptr<YAMLReader> pYAMLReader;

}

bool AppConfig::initialize(const std::string& application_, const std::string& organization_)
{
    application = application_;
    organization = organization_;

#ifdef _WIN32
    const char* appdata = getenv("APPDATA");
    if(appdata){
        configDirPath = filesystem::path(appdata) / organization_;
    }
#else
    const char* home = getenv("HOME");
    if(home){
        configDirPath = filesystem::path(home) / ".config" / organization_;
    }
#endif
    
    filePath = application + ".conf";

    if(!configDirPath.empty()){
        fullPath = configDirPath / filePath;
        std::string fullPathString = getPathString(fullPath);
        load(fullPathString);
    }

    return !fullPath.empty();
}

/**
   \note A pointer returned by archive() must be replaced with a new one
   when a new config is loaed.
*/
Mapping* AppConfig::archive()
{
    if(pYAMLReader && pYAMLReader->numDocuments()){
        return pYAMLReader->document()->toMapping();
    }
    static MappingPtr appArchive(new Mapping);
    return appArchive.get();
}
  

bool AppConfig::flush()
{
    if(configDirPath.empty()){
        return false;
    }
    
    if(!filesystem::exists(fullPath)){
        if(filesystem::exists(configDirPath)){
            if(!filesystem::is_directory(configDirPath)){

                const char* m =
                    "\"{}\" is not a directory.\n"
                    "It should be directory to contain the config file.\n"
                    "The configuration cannot be stored into the file system";
                showWarningDialog(format(_(m),configDirPath.string()));
                return false;
            }
        } else {
            filesystem::create_directories(configDirPath);
        }
    }

    return save(fullPath.string());
}


bool AppConfig::save(const std::string& filename)
{
    try {
        YAMLWriter writer(filename);
        writer.setKeyOrderPreservationMode(true);
        writer.putNode(archive());
    }
    catch(const ValueNode::Exception& ex){
        showWarningDialog(ex.message());
        return false;
    }
    return true;
}


/**
   \note A pointer returned by archive() must be replaced after calling this function
   because a new YAMLReader instance is created in it.
*/
bool AppConfig::load(const std::string& filename)
{
    YAMLReader* pyaml = new YAMLReader();

    if(filesystem::exists(filesystem::path(filename))){
        try {
            if(pyaml->load(filename)){
                if(pyaml->numDocuments() != 1 || !pyaml->document()->isMapping()){
                    pyaml->clearDocuments();
                }
            }
        } catch (const ValueNode::Exception& ex){
            ostream& os = MessageView::mainInstance()->cout();
            os << format("Application config file \"{0}\" cannot be loaded ({1}).",
                    filename, ex.message() ) << endl;
            pyaml->clearDocuments();
            delete pyaml;
            return false;
        }
    }
    
    pYAMLReader = std::shared_ptr<YAMLReader>(pyaml);
    return true;
}
