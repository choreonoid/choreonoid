#include "AbstractSceneWriter.h"
#include "SceneDrawables.h"
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = stdx::filesystem;


AbstractSceneWriter::~AbstractSceneWriter()
{

}


void AbstractSceneWriter::setMessageSink(std::ostream& os)
{
    os_ = &os;
}


bool AbstractSceneWriter::findOrCopyImageFile(SgImage* image, const std::string& outputBaseDir)
{
    bool foundOrCopied = false;
    bool orgImageFileFound = false;
    stdx::error_code ec;
    
    auto uri = image->uri();
    if(uri.find_first_of("file://") == 0){
        uri = uri.substr(7);
    }
    filesystem::path filePath(uri);

    if(filePath.is_absolute()){
        orgImageFileFound = filesystem::exists(filePath, ec);

    } else if(image->hasAbsoluteUri()){
        auto& absUri = image->absoluteUri();
        if(absUri.find_first_of("file://") == 0){
            filesystem::path orgFilePath(absUri.substr(7));
            if(filesystem::exists(orgFilePath, ec)){
                orgImageFileFound = true;
                if(filePath.is_relative()){
                    filePath = filesystem::path(outputBaseDir) / filePath;
                }
                if(filesystem::equivalent(orgFilePath, filePath, ec)){
                    foundOrCopied = true;
                } else {
                    ec.clear();
                    filesystem::create_directories(filePath.parent_path(), ec);
                    if(!ec){
#if __cplusplus > 201402L
                        filesystem::copy_file(
                            orgFilePath, filePath, filesystem::copy_options::update_existing, ec);
#else
                        bool doCopy = true;
                        if(filesystem::exists(filePath, ec)){
                            if(filesystem::last_write_time(filePath, ec) >= filesystem::last_write_time(orgFilePath, ec)){
                                doCopy = false;
                            }
                        }
                        if(doCopy){
                            filesystem::copy_file(
                                orgFilePath, filePath, filesystem::copy_option::overwrite_if_exists, ec);
                        }
#endif
                        if(!ec){
                            foundOrCopied = true;
                        }
                    }
                    if(ec){
                        os() << format(_("Warning: Texture image file \"{0}\" cannot be copied: {1}"),
                                       uri, ec.message()) << endl;
                    }
                }
            }
        }
    }
    
    if(!orgImageFileFound){
        os() << format(_("Warning: Texture image file \"{0}\" is not found."), uri) << endl;
    }

    return foundOrCopied;
}
