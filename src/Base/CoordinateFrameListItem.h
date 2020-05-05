#ifndef CNOID_BASE_COORDINATE_FRAME_LIST_ITEM_H
#define CNOID_BASE_COORDINATE_FRAME_LIST_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameList;
class CoordinateFrameItem;
class LocatableItem;

class CNOID_EXPORT CoordinateFrameListItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    static SignalProxy<void(CoordinateFrameListItem* frameListItem)> sigInstanceAddedOrUpdated();

    CoordinateFrameListItem();
    CoordinateFrameListItem(const CoordinateFrameListItem& org);
    virtual ~CoordinateFrameListItem();

    // Register the callback function called when a new coordinate frame item is created
    void setNewFrameItemCallback(std::function<void(CoordinateFrameItem* item)> callback);

    enum ItemizationMode { NoItemization, SubItemization, IndependentItemization };
    int itemizationMode() const;
    void setItemizationMode(int mode);
    void updateFrameItems();

    CoordinateFrameList* frameList();
    const CoordinateFrameList* frameList() const;

    void useAsBaseFrames();
    void useAsOffsetFrames();
    bool isForBaseFrames() const;
    bool isForOffsetFrames() const;

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
