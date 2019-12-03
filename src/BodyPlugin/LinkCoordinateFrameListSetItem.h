#ifndef CNOID_BODY_PLUGIN_LINK_COORDINATE_FRAME_LIST_SET_ITEM_H
#define CNOID_BODY_PLUGIN_LINK_COORDINATE_FRAME_LIST_SET_ITEM_H

#include <cnoid/MultiCoordinateFrameListItem>
#include <cnoid/LinkCoordinateFrameSet>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkCoordinateFrameListSetItem : public MultiCoordinateFrameListItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    enum FrameType {
        WorldFrame = LinkCoordinateFrameSet::WorldFrame,
        BodyFrame  = LinkCoordinateFrameSet::BodyFrame,
        EndFrame   = LinkCoordinateFrameSet::EndFrame
    };

    static void setFrameListLabels(
        const char* worldFrameLabel, const char* bodyFrameLabel, const char* endFrameLabel);

    static void setFrameListEnabledForAllItems(FrameType type, bool on);

    LinkCoordinateFrameListSetItem();
    LinkCoordinateFrameListSetItem(const LinkCoordinateFrameListSetItem& org);

    LinkCoordinateFrameSet* frameSets();
    const LinkCoordinateFrameSet* frameSets() const;

    CoordinateFrameListItem* worldFrameListItem(int index);
    const CoordinateFrameListItem* worldFrameListItem(int index) const;

    CoordinateFrameListItem* bodyFrameListItem(int index);
    const CoordinateFrameListItem* bodyFrameListItem(int index) const;

    CoordinateFrameListItem* endFrameListItem(int index);
    const CoordinateFrameListItem* endFrameListItem(int index) const;

protected:
    virtual Item* doDuplicate() const override;

private:
    void initializeFrameListEnabling();
    
    ScopedConnection connection;
};

typedef ref_ptr<LinkCoordinateFrameListSetItem> LinkCoordinateFrameListSetItemItemPtr;

}

#endif
