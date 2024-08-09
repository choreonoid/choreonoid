#include "AbstractSceneWriter.h"
#include "SceneDrawables.h"
#include "UTF8.h"
#include "Format.h"
#include <cnoid/stdx/filesystem>
#include <unordered_map>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = stdx::filesystem;

namespace cnoid {

class AbstractSceneWriter::Impl
{
public:
    struct CopiedFileInfo {
        string filename;
        bool copied;
    };
    
    // Original file to copied file
    unordered_map<string, CopiedFileInfo> imageFileMap;
    unordered_set<string> copiedImageFiles;
};

}


AbstractSceneWriter::AbstractSceneWriter()
{
    impl = new Impl;
}


AbstractSceneWriter::~AbstractSceneWriter()
{
    delete impl;
}


void AbstractSceneWriter::setMessageSink(std::ostream& os)
{
    os_ = &os;
}


void AbstractSceneWriter::clearImageFileInformation()
{
    impl->imageFileMap.clear();
    impl->copiedImageFiles.clear();
}


bool AbstractSceneWriter::findOrCopyImageFile(SgImage* image, const std::string& outputBaseDir, std::string& out_copiedFile)
{
    bool foundOrCopied = false;
    bool orgImageFileFound = false;
    stdx::error_code ec;
    
    auto uri = image->uri();
    if(uri.find("file://") == 0){
        uri = uri.substr(7);
    }
    filesystem::path filePath(fromUTF8(uri));

    if(filePath.is_absolute()){
        orgImageFileFound = filesystem::exists(filePath, ec);
        if(orgImageFileFound){
            out_copiedFile = uri;
            foundOrCopied = true;
        }
    } else if(image->hasAbsoluteUri()){
        auto& absUri = image->absoluteUri();
        if(absUri.find("file://") == 0){
            filesystem::path orgFilePath(fromUTF8(absUri.substr(7)));
            auto found = impl->imageFileMap.find(orgFilePath.string());
            if(found != impl->imageFileMap.end()){
                auto& info = found->second;
                out_copiedFile = toUTF8(info.filename);
                orgImageFileFound = true;
                foundOrCopied = info.copied;

            } else if(filesystem::exists(orgFilePath, ec)){
                orgImageFileFound = true;
                filesystem::path absPath;
                if(filePath.is_relative()){
                    absPath = filesystem::path(fromUTF8(outputBaseDir)) / filePath;
                } else {
                    absPath = filePath;
                }
                auto stem = filePath.stem().string();
                auto ext = filePath.extension().string();
                int counter = 2;
                while(true){
                    auto inserted = impl->copiedImageFiles.insert(absPath.string());
                    if(inserted.second){
                        break;
                    }
                    filePath = filePath.parent_path() / formatC("{0}-{1}{2}", stem, counter, ext);
                    ++counter;
                    if(filePath.is_relative()){
                        absPath = filesystem::path(fromUTF8(outputBaseDir)) / filePath;
                    } else {
                        absPath = filePath;
                    }
                }
                if(filesystem::equivalent(orgFilePath, absPath, ec)){
                    foundOrCopied = true;
                    out_copiedFile = toUTF8(filePath.string());
                } else {
                    filesystem::create_directories(absPath.parent_path(), ec);
                    if(!ec){
#if __cplusplus > 201402L
                        filesystem::copy_file(
                            orgFilePath, absPath, filesystem::copy_options::overwrite_existing, ec);
#else
                        filesystem::copy_file(
                            orgFilePath, absPath, filesystem::copy_option::overwrite_if_exists, ec);
#endif
                    }
                    if(!ec){
                        foundOrCopied = true;
                        out_copiedFile = toUTF8(filePath.string());
                    }
                }
                if(ec){
                    os() << formatR(_("Warning: Texture image file \"{0}\" cannot be copied: {1}"),
                                    uri, ec.message()) << endl;
                }

                auto& info = impl->imageFileMap[orgFilePath.string()];
                info.filename = filePath.string();
                info.copied = foundOrCopied;
            }
        }
    }
    
    if(!orgImageFileFound){
        os() << formatR(_("Warning: Texture image file \"{0}\" is not found."), uri) << endl;
    }

    return foundOrCopied;
}
