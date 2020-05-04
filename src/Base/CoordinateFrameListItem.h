#ifndef CNOID_BASE_COORDINATE_FRAME_LIST_ITEM_H
#define CNOID_BASE_COORDINATE_FRAME_LIST_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameList;
class LocatableItem;

class CNOID_EXPORT CoordinateFrameListItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    static SignalProxy<void(CoordinateFrameListItem* frameListItem)> sigInstanceAddedOrUpdated();

    CoordinateFrameListItem();
    CoordinateFrameListItem(const CoordinateFrameListItem& org);
    virtual ~CoordinateFrameListItem();

    enum ItemizationMode { NoItemization, SubItemization, IndependentItemization };
    int itemizationMode() const;
    void setItemizationMode(int mode);

    CoordinateFrameList* frameList();
    const CoordinateFrameList* frameList() const;

    virtual LocatableItem* getParentLocatableItem();

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;
    virtual bool onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameListItem> CoordinateFrameListItemPtr;

}

#endif
