#ifndef CNOID_BODY_PLUGIN_BODY_OVERWRITE_ADDON_H
#define CNOID_BODY_PLUGIN_BODY_OVERWRITE_ADDON_H

#include "DeviceOverwriteItem.h"
#include <cnoid/ItemAddon>
#include <cnoid/ItemList>
#include "exportdecl.h"

namespace cnoid {

class LinkShapeOverwriteItem;
class DeviceOverwriteItem;
class Link;
class Device;

class CNOID_EXPORT BodyOverwriteAddon : public ItemAddon
{
public:
    static void initializeClass(ExtensionManager* ext);

    BodyOverwriteAddon();
    BodyOverwriteAddon(const BodyOverwriteAddon*) = delete;
    
    virtual bool setOwnerItem(Item* item) override;

    // Functions on link shape overwriting
    bool checkIfSingleShapeBody();
    LinkShapeOverwriteItem* findLinkShapeOverwriteItem(Link* link);
    LinkShapeOverwriteItem* getOrCreateLinkShapeOverwriteItem(Link* link);
    bool registerLinkShapeOverwriteItem(Link* link, LinkShapeOverwriteItem* item);
    void unregisterLinkShapeOverwriteItem(LinkShapeOverwriteItem* item);

    // Functions on device overwriting

    // Note that the getter ids of newly extracted items are not specified by this function and
    // the ids should be specified by a caller after calling this function.
    ItemList<DeviceOverwriteItem> extractDevicesAsOverwriteItems(std::function<bool(Device* device)> predicate);

    template<class DeviceType>
    ItemList<DeviceOverwriteItem> deviceOverwriteItems(){
        return ownerItem()->descendantItems<DeviceOverwriteItem>(
            [](DeviceOverwriteItem* item) -> bool {
                return dynamic_cast<DeviceType*>(item->device());
            });
    }
    
    DeviceOverwriteItem* findDeviceOverwriteItem(Device* device);
    bool addDeviceOverwriteItem(DeviceOverwriteItem* item);
    void removeDeviceOverwriteItem(DeviceOverwriteItem* item);
    
    void clearOverwriteItems();

private:
    class Impl;
    Impl* impl;
};

}

#endif
