/*!
  @file
  @author Shizuko Hattori
*/

#ifndef CNOID_BASE_LIGHTING_ITEM_H
#define CNOID_BASE_LIGHTING_ITEM_H

#include "Item.h"
#include <cnoid/SceneProvider>
#include "exportdecl.h"

namespace cnoid {

class LightingItemImpl;

class CNOID_EXPORT LightingItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    LightingItem();
    LightingItem(const LightingItem& org);
    virtual ~LightingItem();

protected:
    virtual Item* doDuplicate() const;
    virtual SgNode* getScene() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    LightingItemImpl* impl;
};

typedef ref_ptr<LightingItem> LightingItemPtr;

}

#endif
