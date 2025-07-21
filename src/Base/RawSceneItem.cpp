#include "RawSceneItem.h"
#include "ItemManager.h"
#include <cnoid/SceneGraph>
#include "gettext.h"

using namespace std;
using namespace cnoid;

void RawSceneItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<RawSceneItem>(N_("RawSceneItem"));
}


RawSceneItem::RawSceneItem()
{

}


RawSceneItem::RawSceneItem(const RawSceneItem& org, CloneMap* cloneMap)
    : Item(org)
{
    if(cloneMap){
        scene_ = org.scene_->cloneNode(*cloneMap);
    } else {
        scene_ = org.scene_;
    }
}


Item* RawSceneItem::doCloneItem(CloneMap* cloneMap) const
{
    return new RawSceneItem(*this, cloneMap);
}


void RawSceneItem::setScene(SgNode* node)
{
    scene_ = node;
}


SgNode* RawSceneItem::getScene()
{
    return scene_;
}
