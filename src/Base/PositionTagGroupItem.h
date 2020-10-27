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
    bool isOriginMarkerVisible() const;
    void setOriginMarkerVisible(bool on);

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;

    const Position& parentPosition() const;
    Position globalOriginOffset() const;

    double tagMarkerSize() const;
    void setTagMarkerSize(double s);

    bool edgeVisibility() const;
    void setEdgeVisiblility(bool on);

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual bool onNewPositionCheck(bool isManualOperation, std::function<void()>& out_callbackWhenAdded) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    Impl* impl;
};

typedef ref_ptr<PositionTagGroupItem> PositionTagGroupItemPtr;

}

#endif
