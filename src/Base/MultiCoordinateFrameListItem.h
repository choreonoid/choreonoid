#ifndef CNOID_BASE_MULTI_COORDINATE_FRAME_LIST_ITEM_H
#define CNOID_BASE_MULTI_COORDINATE_FRAME_LIST_ITEM_H

#include <cnoid/Item>
#include <initializer_list>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameList;
class CoordinateFrameListItem;
class MultiCoordinateFrameSet;

class CNOID_EXPORT MultiCoordinateFrameListItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    MultiCoordinateFrameListItem();
    MultiCoordinateFrameListItem(std::initializer_list<std::string> frameSetNames);
    MultiCoordinateFrameListItem(const MultiCoordinateFrameListItem& org);
    virtual ~MultiCoordinateFrameListItem();

    MultiCoordinateFrameSet* frameSets();
    const MultiCoordinateFrameSet* frameSets() const;

    int numFrameLists() const;
    CoordinateFrameList* frameList(int index);
    const CoordinateFrameList* frameList(int index) const;

    CoordinateFrameListItem* frameListItem(int index);
    const CoordinateFrameListItem* frameListItem(int index) const;

    void setFrameListEnabled(int index, bool on);
    bool isFrameListEnabled(int index) const;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;
    void replaceFrameListContainer(MultiCoordinateFrameSet* container);

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MultiCoordinateFrameListItem> MultiCoordinateFrameListItemItemPtr;

}

#endif
