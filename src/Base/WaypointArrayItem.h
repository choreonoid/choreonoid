#ifndef CNOID_BASE_WAYPOINT_ARRAY_ITEM_H
#define CNOID_BASE_WAYPOINT_ARRAY_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include "LocatableItem.h"
#include "exportdecl.h"

namespace cnoid {

class WaypointArray;

class CNOID_EXPORT WaypointArrayItem : public Item, public RenderableItem, public LocatableItem
{
public:
    //static void initializeClass(ExtensionManager* ext);

    WaypointArrayItem();
    WaypointArrayItem(const WaypointArrayItem& org);
    virtual ~WaypointArrayItem();

    const WaypointArray* waypoints() const;
    WaypointArray* waypoints();

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

typedef ref_ptr<WaypointArrayItem> WaypointArrayItemPtr;

}

#endif
