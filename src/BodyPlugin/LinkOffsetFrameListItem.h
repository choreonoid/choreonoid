#ifndef CNOID_BODY_PLUGIN_LINK_OFFSET_FRAME_LIST_ITEM_H
#define CNOID_BODY_PLUGIN_LINK_OFFSET_FRAME_LIST_ITEM_H

#include <cnoid/CoordinateFrameListItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkOffsetFrameListItem : public CoordinateFrameListItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    LinkOffsetFrameListItem();
    LinkOffsetFrameListItem(CoordinateFrameList* frameList);
    ~LinkOffsetFrameListItem();

    virtual LocationProxyPtr getFrameParentLocationProxy() override;

protected:
    LinkOffsetFrameListItem(const LinkOffsetFrameListItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<LinkOffsetFrameListItem> LinkOffsetFrameListItemPtr;

}

#endif
