#include "RenderableItemUtil.h"
#include "Item.h"
#include "RenderableItem.h"
#include "ProjectManager.h"
#include <cnoid/SceneGraph>
#include <cnoid/FileUtil>
#include <cnoid/UTF8>
#include <cnoid/MessageOut>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace fs = stdx::filesystem;

namespace cnoid {

class RenderableItemUtil::Impl
{
public:
    Item* item;
    MessageOut* mout;
    fs::path itemFilePath;
    fs::path baseDirPath;
    set<SgObject*> relocatedSceneObjects;

    Impl();
    void getSceneFiles(SgObject* object, std::vector<std::string>& out_files);
    bool getSceneFilesForArchiving(SgObject* object, std::vector<std::string>& out_files);
    void relocateSceneObjectUris(
        SgObject* object, const std::function<std::string(const std::string& path)>& getRelocatedFilePath);
};

}


RenderableItemUtil::RenderableItemUtil()
{
    impl = new Impl;
}


RenderableItemUtil::Impl::Impl()
{
    item = nullptr;
    mout = nullptr;
}


void RenderableItemUtil::setItem(Item* item)
{
    impl->item = item;
}


void RenderableItemUtil::setMessageOut(MessageOut* mout)
{
    impl->mout = mout;
}


bool RenderableItemUtil::getSceneFilesForArchiving(std::vector<std::string>& out_files)
{
    if(auto renderableItem = dynamic_cast<RenderableItem*>(impl->item)){
        return impl->getSceneFilesForArchiving(renderableItem->getScene(), out_files);
    }
    return false;
}


bool RenderableItemUtil::getSceneFilesForArchiving(SgObject* object, std::vector<std::string>& out_files)
{
    return impl->getSceneFilesForArchiving(object, out_files);
}


bool RenderableItemUtil::Impl::getSceneFilesForArchiving(SgObject* object, std::vector<std::string>& out_files)
{
    if(!mout){
        mout = MessageOut::master();
    }
    
    baseDirPath.clear();
    itemFilePath = getNativeUniformPath(fromUTF8(item->filePath()));
    if(!itemFilePath.empty()){
        baseDirPath = itemFilePath;
        baseDirPath.remove_filename();
    } else {
        baseDirPath = fromUTF8(ProjectManager::instance()->currentProjectDirectory());
        if(baseDirPath.empty()){
            mout->putErrorln(
                format(_("The project must be saved to detect the scene files for {0}."),
                       item->displayName()));
            return false;
        }
    }

    getSceneFiles(object, out_files);

    return true;
}


void RenderableItemUtil::Impl::getSceneFiles(SgObject* object, std::vector<std::string>& out_files)
{
    string filePath = object->localFilePath();
    string absolutePath = object->localFileAbsolutePath();
    fs::path uniformPath;
    bool isAbsolutePathOnly = false;
                          
    if(!filePath.empty()){
        if(fs::path(fromUTF8(filePath)).is_absolute()){
            isAbsolutePathOnly = true;
        } else {
            uniformPath = getNativeUniformPath(fromUTF8(absolutePath));
            if(uniformPath != itemFilePath){
                out_files.push_back(absolutePath);
            }
        }
    } else if(object->hasAbsoluteUri()){
        isAbsolutePathOnly = true;
    }
    
    if(isAbsolutePathOnly){
        if(uniformPath.empty()){
            uniformPath = getNativeUniformPath(fromUTF8(absolutePath));
        }
        if(uniformPath != itemFilePath){
            mout->putWarningln(
                format(_("The file \"{0}\" on which {1} depends is not relocatable and will be lost in the archive."),
                       object->absoluteUri(), item->displayName()));
        }
    }

    int n = object->numChildObjects();
    for(int i=0; i < n; ++i){
        getSceneFiles(object->childObject(i), out_files);
    }
}


void RenderableItemUtil::initializeSceneObjectUrlRelocation()
{
    impl->relocatedSceneObjects.clear();
}


void RenderableItemUtil::relocateSceneObjectUris(std::function<std::string(const std::string& path)> getRelocatedFilePath)
{
    if(auto renderableItem = dynamic_cast<RenderableItem*>(impl->item)){
        impl->relocateSceneObjectUris(renderableItem->getScene(), getRelocatedFilePath);
    }
}


void RenderableItemUtil::relocateSceneObjectUris
(SgObject* object, std::function<std::string(const std::string& path)> getRelocatedFilePath)
{
    impl->relocateSceneObjectUris(object, getRelocatedFilePath);
}


void RenderableItemUtil::Impl::relocateSceneObjectUris
(SgObject* object, const std::function<std::string(const std::string& path)>& getRelocatedFilePath)
{
    string absolutePath = object->localFileAbsolutePath();
    if(!absolutePath.empty()){
        bool inserted = relocatedSceneObjects.insert(object).second;
        if(inserted){
            auto relocated = getRelocatedFilePath(absolutePath);
            if(!relocated.empty()){
                string filePath = object->localFilePath();
                if(fs::path(fromUTF8(filePath)).is_absolute()){
                    object->setUri(relocated, relocated);
                } else {
                    object->setUri(filePath, relocated);
                }
            }
        }
    }

    int n = object->numChildObjects();
    for(int i=0; i < n; ++i){
        relocateSceneObjectUris(object->childObject(i), getRelocatedFilePath);
    }
}
