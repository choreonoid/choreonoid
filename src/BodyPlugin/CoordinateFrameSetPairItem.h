#ifndef CNOID_BODY_PLUGIN_COORDINATE_FRAME_SET_PAIR_ITEM_H
#define CNOID_BODY_PLUGIN_COORDINATE_FRAME_SET_PAIR_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameSetItem;
class CoordinateFrameContainer;
class CoordinateFrameSetPair;

class CNOID_EXPORT CoordinateFrameSetPairItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    static void setFrameSetLabels(const char* baseFrameLabel, const char* localFrameLabeld);

    CoordinateFrameSetPairItem();
    CoordinateFrameSetPairItem(const CoordinateFrameSetPairItem& org);
    virtual ~CoordinateFrameSetPairItem();

    CoordinateFrameSetPair* frameSetPair();
    const CoordinateFrameSetPair* frameSetPair() const;

    CoordinateFrameSetItem* baseFrameSetItem();
    const CoordinateFrameSetItem* baseFrameSetItem() const;
    CoordinateFrameSetItem* localFrameSetItem();
    const CoordinateFrameSetItem* localFrameSetItem() const;

    CoordinateFrameContainer* baseFrames();
    const CoordinateFrameContainer* baseFrames() const;
    CoordinateFrameContainer* localFrames();
    const CoordinateFrameContainer* localFrames() const;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameSetPairItem> CoordinateFrameSetPairItemItemPtr;

}

#endif
