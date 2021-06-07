/**
   @author Shin'ichiro Nakaoka
*/

#include "AppConfig.h"
#include "MessageView.h"
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
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

MappingPtr configArchive;

}

static bool loadConfig(const string& filename);
static void putLoadError(const string& filename, const string& message);


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
    
    filePath = fromUTF8(application + ".conf");

    bool loaded = false;
    if(!configDirPath.empty()){
        fullPath = configDirPath / filePath;
        loaded = loadConfig(toUTF8(fullPath.string()));
    }

    if(!loaded){
        configArchive = new Mapping;
    }

    return loaded;
}


static bool loadConfig(const std::string& filename)
{
    bool loaded = false;

    if(filesystem::exists(fromUTF8(filename))){
        YAMLReader reader;
        if(!reader.load(filename)){
            putLoadError(filename, reader.errorMessage());
        } else {
            if(reader.numDocuments() == 1 && reader.document()->isMapping()){
                configArchive = reader.document()->toMapping();
                loaded = true;
            } else {
                putLoadError(filename, _("Invalid file format."));
            }
        }
    }

    return loaded;
}


static void putLoadError(const string& filename, const string& message)
{
    MessageView::postMessageBeforeInitialization(
        format(_("Application config file \"{0}\" cannot be loaded.\n{1}"), filename, message),
        MessageView::Error);
}


Mapping* AppConfig::archive()
{
    return configArchive;
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
                showWarningDialog(format(_(m), configDirPath.string()));
                return false;
            }
        } else {
            filesystem::create_directories(configDirPath);
        }
    }

    try {
        YAMLWriter writer(toUTF8(fullPath.string()));
        writer.setKeyOrderPreservationMode(true);
        writer.putNode(configArchive);
    }
    catch(const ValueNode::Exception& ex){
        showWarningDialog(ex.message());
        return false;
    }
    
    return true;
}
