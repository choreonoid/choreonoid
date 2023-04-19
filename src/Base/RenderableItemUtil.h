#ifndef CNOID_BASE_RENDERABLE_ITEM_UTIL_H
#define CNOID_BASE_RENDERABLE_ITEM_UTIL_H

#include <vector>
#include <string>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Item;
class MessageOut;
class SgObject;

class CNOID_EXPORT RenderableItemUtil
{
public:
    RenderableItemUtil();
    void setItem(Item* item);
    void setMessageOut(MessageOut* mout);
    bool getSceneFilesForArchiving(std::vector<std::string>& out_files);
    bool getSceneFilesForArchiving(SgObject* object, std::vector<std::string>& out_files);
    void initializeSceneObjectUrlRelocation();
    void relocateSceneObjectUris(std::function<std::string(const std::string& path)> getRelocatedFilePath);
    void relocateSceneObjectUris(
        SgObject* object, std::function<std::string(const std::string& path)> getRelocatedFilePath);

private:
    class Impl;
    Impl* impl;
};

}

#endif
