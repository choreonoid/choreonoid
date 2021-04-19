#ifndef CNOID_BASE_COORDINATE_FRAME_ITEM_H
#define CNOID_BASE_COORDINATE_FRAME_ITEM_H

#include "Item.h"
#include "LocatableItem.h"
#include <cnoid/GeneralId>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrame;
class CoordinateFrameList;
class CoordinateFrameListItem;

/**
   \note This item is always used as a child item of CoordinateFrameListItem
*/
class CNOID_EXPORT CoordinateFrameItem : public Item, public LocatableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    CoordinateFrameItem();
    CoordinateFrameItem(CoordinateFrame* frame);
    CoordinateFrameItem(const CoordinateFrameItem& org);
    virtual ~CoordinateFrameItem();

    virtual std::string displayName() const override;

    CoordinateFrameListItem* frameListItem();
    CoordinateFrameList* frameList();
    const CoordinateFrameList* frameList() const;
    CoordinateFrame* frame();
    const CoordinateFrame* frame() const;

    /**
       This function emits CoordinateFrame::sigUpdated when the id is successfully changed.
    */
    bool resetFrameId(const GeneralId& id);
    
    bool isBaseFrame() const;
    bool isOffsetFrame() const;

    void setVisibilityCheck(bool on);

    void putFrameAttributes(PutPropertyFunction& putProperty);

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;

    bool isLocationEditable() const;
    void setLocationEditable(bool on);
    
    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onAddedToParent() override;
    virtual void onRemovedFromParent(Item* parentItem, bool isParentBeingDeleted) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameItem> CoordinateFrameItemPtr;

}

#endif
