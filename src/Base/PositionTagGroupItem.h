#ifndef CNOID_BASE_POSITION_TAG_GROUP_ITEM_H
#define CNOID_BASE_POSITION_TAG_GROUP_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include "LocatableItem.h"
#include "exportdecl.h"

namespace cnoid {

class PositionTagGroup;

class CNOID_EXPORT PositionTagGroupItem : public Item, public RenderableItem, public LocatableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    PositionTagGroupItem();
    PositionTagGroupItem(const PositionTagGroupItem& org);
    virtual ~PositionTagGroupItem();

    const PositionTagGroup* tags() const;
    PositionTagGroup* tags();

    // RenderableItem function
    virtual SgNode* getScene() override;

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;

    const Position& parentPosition() const;
    Position globalOffsetPosition() const;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;

private:
    Impl* impl;
};

typedef ref_ptr<PositionTagGroupItem> PositionTagGroupItemPtr;

}

#endif
