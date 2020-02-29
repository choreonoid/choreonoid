#ifndef CNOID_BODY_PLUGIN_LINK_COORD_FRAME_LIST_SUITE_ITEM_H
#define CNOID_BODY_PLUGIN_LINK_COORD_FRAME_LIST_SUITE_ITEM_H

#include <cnoid/CoordinateFrameListSuiteItem>
#include <cnoid/LinkCoordFrameSetSuite>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkCoordFrameListSuiteItem : public CoordinateFrameListSuiteItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    static SignalProxy<void(LinkCoordFrameListSuiteItem* frameListSetItem)> sigInstanceAddedOrUpdated();

    enum FrameType {
        WorldFrame = LinkCoordFrameSetSuite::WorldFrame,
        BodyFrame  = LinkCoordFrameSetSuite::BodyFrame,
        LinkFrame   = LinkCoordFrameSetSuite::LinkFrame
    };

    static void setFrameListLabels(
        const char* worldFrameLabel, const char* bodyFrameLabel, const char* linkFrameLabel);

    static void setFrameListEnabledForAllItems(FrameType type, bool on);

    LinkCoordFrameListSuiteItem();
    LinkCoordFrameListSuiteItem(const LinkCoordFrameListSuiteItem& org);

    LinkCoordFrameSetSuite* frameSetSuite();
    const LinkCoordFrameSetSuite* frameSetSuite() const;

    CoordinateFrameListItem* worldFrameListItem(int index);
    const CoordinateFrameListItem* worldFrameListItem(int index) const;

    CoordinateFrameListItem* bodyFrameListItem(int index);
    const CoordinateFrameListItem* bodyFrameListItem(int index) const;

    CoordinateFrameListItem* linkFrameListItem(int index);
    const CoordinateFrameListItem* linkFrameListItem(int index) const;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;

private:
    void initializeFrameListEnabling();
    
    ScopedConnection connection;
};

typedef ref_ptr<LinkCoordFrameListSuiteItem> LinkCoordFrameListSuiteItemItemPtr;

}

#endif
