#ifndef CNOID_MANIPULATOR_PLUGIN_COORDINATE_FRAME_ITEM_H
#define CNOID_MANIPULATOR_PLUGIN_COORDINATE_FRAME_ITEM_H

#include <cnoid/CoordinateFrameSetPairItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ManipulatorCoordinateFrameItem : public CoordinateFrameSetPairItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    ManipulatorCoordinateFrameItem();
    ManipulatorCoordinateFrameItem(const ManipulatorCoordinateFrameItem& org);

    CoordinateFrameSetItem* toolFrameSetItem(){ return localFrameSetItem(); }
    const CoordinateFrameSetItem* toolFrameSetItem() const { return localFrameSetItem(); }

    CoordinateFrameContainer* toolFrames(){ return localFrames(); }
    const CoordinateFrameContainer* toolFrames() const { return localFrames(); }

protected:
    virtual Item* doDuplicate() const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<ManipulatorCoordinateFrameItem> ManipulatorCoordinateFrameItemPtr;

}

#endif
