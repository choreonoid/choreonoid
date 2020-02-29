#ifndef CNOID_BASE_COORDINATE_FRAME_LIST_SUITE_ITEM_H
#define CNOID_BASE_COORDINATE_FRAME_LIST_SUITE_ITEM_H

#include <cnoid/Item>
#include <initializer_list>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrameList;
class CoordinateFrameListItem;
class CoordinateFrameSetSuite;

class CNOID_EXPORT CoordinateFrameListSuiteItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    CoordinateFrameListSuiteItem();
    CoordinateFrameListSuiteItem(std::initializer_list<std::string> frameSetNames);
    CoordinateFrameListSuiteItem(const CoordinateFrameListSuiteItem& org);
    virtual ~CoordinateFrameListSuiteItem();

    CoordinateFrameSetSuite* frameSetSuite();
    const CoordinateFrameSetSuite* frameSetSuite() const;

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
    void setNumFrameLists(int n);
    void setFrameListItem(int index, CoordinateFrameListItem* item);
    void replaceFrameListContainer(CoordinateFrameSetSuite* container);

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameListSuiteItem> CoordinateFrameListSuiteItemItemPtr;

}

#endif
