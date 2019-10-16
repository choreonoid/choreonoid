#ifndef CNOID_BASE_COORDINATE_FRAME_LIST_PAIR_ITEM_H
#define CNOID_BASE_COORDINATE_FRAME_LIST_PAIR_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameList;
class CoordinateFrameListItem;
class CoordinateFrameSetPair;

class CNOID_EXPORT CoordinateFrameListPairItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    static void setFrameListLabels(const char* baseFrameLabel, const char* localFrameLabeld);

    CoordinateFrameListPairItem();
    CoordinateFrameListPairItem(const CoordinateFrameListPairItem& org);
    virtual ~CoordinateFrameListPairItem();

    CoordinateFrameSetPair* frameSetPair();
    const CoordinateFrameSetPair* frameSetPair() const;

    CoordinateFrameListItem* baseFrameListItem();
    const CoordinateFrameListItem* baseFrameListItem() const;
    CoordinateFrameListItem* localFrameListItem();
    const CoordinateFrameListItem* localFrameListItem() const;

    CoordinateFrameList* baseFrames();
    const CoordinateFrameList* baseFrames() const;
    CoordinateFrameList* localFrames();
    const CoordinateFrameList* localFrames() const;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameListPairItem> CoordinateFrameListPairItemItemPtr;

}

#endif
