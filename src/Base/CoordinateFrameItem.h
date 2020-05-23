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

    virtual std::string displayName() const;

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

    // LocatableItem functions
    virtual int getLocationType() const override;
    virtual LocatableItem* getParentLocatableItem() override;
    virtual std::string getLocationName() const override;
    virtual Position getLocation() const override;
    virtual bool isLocationEditable() const override;
    virtual void setLocation(const Position& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onAttachedToParent() override;
    virtual void onDetachedFromParent() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameItem> CoordinateFrameItemPtr;

}

#endif
