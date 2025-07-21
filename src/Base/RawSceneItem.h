#ifndef CNOID_BASE_RAW_SCENE_ITEM_H
#define CNOID_BASE_RAW_SCENE_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include "exportdecl.h"

namespace cnoid {

class SgNode;
typedef ref_ptr<SgNode> SgNodePtr;

/**
   This item class is currently only used as a temporary item.
*/
class CNOID_EXPORT RawSceneItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    RawSceneItem();

    void setScene(SgNode* node);

    // RenderableItem
    virtual SgNode* getScene() override;

protected:
    RawSceneItem(const RawSceneItem& org, CloneMap* cloneMap);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;

private:
    SgNodePtr scene_;
};

typedef ref_ptr<RawSceneItem> RawSceneItemPtr;

}

#endif
