#ifndef CNOID_BASE_WAYPOINT_LIST_ITEM_H
#define CNOID_BASE_WAYPOINT_LIST_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include "LocatableItem.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT WaypointListItem : public Item, public RenderableItem, public LocatableItem
{
public:
    //static void initializeClass(ExtensionManager* ext);

    WaypointListItem();
    WaypointListItem(const WaypointListItem& org);
    virtual ~WaypointListItem();

    // RenderableItem function
    virtual SgNode* getScene() override;

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;

private:
    Impl* impl;
};

typedef ref_ptr<WaypointListItem> WaypointListItemPtr;

}

#endif
