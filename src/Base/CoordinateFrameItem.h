#ifndef CNOID_BASE_COORDINATE_FRAME_ITEM_H
#define CNOID_BASE_COORDINATE_FRAME_ITEM_H

#include <cnoid/Item>
#include <cnoid/GeneralId>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrame;
class CoordinateFrameList;

/**
   \note This item is not independently used, but used as a child item
   of CoordinateFrameListItem
*/
class CNOID_EXPORT CoordinateFrameItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    CoordinateFrameItem();
    CoordinateFrameItem(const CoordinateFrameItem& org);
    virtual ~CoordinateFrameItem();

    void setFrameId(const GeneralId& id);
    const GeneralId& frameId() const;
    CoordinateFrameList* frameList();

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;

    //virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameItem> CoordinateFrameItemPtr;

}

#endif
