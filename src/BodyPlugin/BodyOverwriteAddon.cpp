#include "BodyOverwriteAddon.h"
#include "LinkOverwriteItem.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/SceneDrawables>
#include <cnoid/Archive>
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


namespace cnoid {

class BodyOverwriteAddon::Impl
{
public:
    BodyOverwriteAddon* self;
    BodyItem* bodyItem;

    std::map<LinkPtr, LinkOverwriteItemPtr> linkOverwriteItemMap;
    std::map<DevicePtr, DeviceOverwriteItemPtr> deviceOverwriteItemMap;

    Impl(BodyOverwriteAddon* self);
    Impl(BodyOverwriteAddon* self, const BodyOverwriteAddon& org, CloneMap* cloneMap);
};

}


void BodyOverwriteAddon::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerAddon<BodyOverwriteAddon>("BodyOverwrite");
}


BodyOverwriteAddon::BodyOverwriteAddon()
{
    impl = new Impl(this);
}


BodyOverwriteAddon::Impl::Impl(BodyOverwriteAddon* self)
    : self(self)
{
    bodyItem = nullptr;
}


BodyOverwriteAddon::BodyOverwriteAddon(const BodyOverwriteAddon& org, CloneMap* cloneMap)
    : ItemAddon(org)
{
    impl = new Impl(this, org, cloneMap);
}


BodyOverwriteAddon::Impl::Impl(BodyOverwriteAddon* self, const BodyOverwriteAddon& org, CloneMap* cloneMap)
    : self(self)
{
    bodyItem = nullptr;

    if(cloneMap){
        for(auto& kv : org.impl->linkOverwriteItemMap){
            auto& link = kv.first;
            auto& item = kv.second;
            if(auto linkClone = cloneMap->findClone(link)){
                cloneMap->findCloneOrReplaceLater<LinkOverwriteItem>(
                    item,
                    [this, linkClone](LinkOverwriteItem* itemClone){
                        linkOverwriteItemMap[linkClone] = itemClone;
                    });
            }
        }
        for(auto& kv : org.impl->deviceOverwriteItemMap){
            auto& device = kv.first;
            auto& item = kv.second;
            if(auto deviceClone = cloneMap->findClone(device)){
                cloneMap->findCloneOrReplaceLater<DeviceOverwriteItem>(
                    item,
                    [this, deviceClone](DeviceOverwriteItem* itemClone){
                        deviceOverwriteItemMap[deviceClone] = itemClone;
                    });
            }
        }
    }
}


ItemAddon* BodyOverwriteAddon::doClone(Item* /* newItem */, CloneMap* cloneMap) const
{
    return new BodyOverwriteAddon(*this, cloneMap);
}


bool BodyOverwriteAddon::setOwnerItem(Item* item)
{
    impl->bodyItem = dynamic_cast<BodyItem*>(item);
    ItemAddon::setOwnerItem(impl->bodyItem);
    return bool(impl->bodyItem);
}


LinkOverwriteItem* BodyOverwriteAddon::findLinkOverwriteItem(Link* link)
{
    auto p = impl->linkOverwriteItemMap.find(link);
    if(p != impl->linkOverwriteItemMap.end()){
        return p->second;
    }
    return nullptr;
}


bool BodyOverwriteAddon::registerLinkOverwriteItem(Link* link, LinkOverwriteItem* item)
{
    auto body = link->body();
    if(!body || (body == impl->bodyItem->body())){
        auto& mappedItem = impl->linkOverwriteItemMap[link];
        if(!mappedItem){
            mappedItem = item;
            return true;
        }
    }
    return false;
}


void BodyOverwriteAddon::unregisterLinkOverwriteItem(LinkOverwriteItem* item)
{
    if(item->bodyItem() == impl->bodyItem){
        auto p = impl->linkOverwriteItemMap.begin();
        while(p != impl->linkOverwriteItemMap.end()){
            if(p->second == item){
                impl->linkOverwriteItemMap.erase(p);
                break;
            }
            ++p;
        }
    }
}


ItemList<DeviceOverwriteItem> BodyOverwriteAddon::extractDevicesAsOverwriteItems
(std::function<bool(Device* device)> predicate)
{
    ItemList<DeviceOverwriteItem> items;
    auto body = impl->bodyItem->body();
    int n = body->numDevices();
    items.reserve(n);
    Item* prevItem = nullptr;
    for(int i=0; i < n; ++i){
        auto device = body->device(i);
        bool isTarget = predicate(device);
        if(isTarget){
            DeviceOverwriteItemPtr item = findDeviceOverwriteItem(device);
            if(!item){
                item = new DeviceOverwriteItem;
                if(device->name().empty()){
                    item->setName(device->typeName());
                } else {
                    item->setName(device->name());
                } 
                if(!item->setDevice(impl->bodyItem, device, false)){
                    item.reset();
                } else {
                    if(prevItem){
                        impl->bodyItem->insertChild(prevItem->nextItem(), item);
                    } else {
                        impl->bodyItem->addChildItem(item);
                    }
                }
            }
            if(item){
                items.push_back(item);
                prevItem = item;
            }
        }
    }
    return items;                
}


DeviceOverwriteItem* BodyOverwriteAddon::findDeviceOverwriteItem(Device* device)
{
    auto p = impl->deviceOverwriteItemMap.find(device);
    if(p != impl->deviceOverwriteItemMap.end()){
        return p->second;
    }
    return nullptr;
}


bool BodyOverwriteAddon::registerDeviceOverwriteItem(DeviceOverwriteItem* item)
{
    if(auto device = item->device()){
        impl->deviceOverwriteItemMap[device] = item;
        return true;
    }
    return false;
}


void BodyOverwriteAddon::unregisterDeviceOverwriteItem(DeviceOverwriteItem* item)
{
    auto p = impl->deviceOverwriteItemMap.find(item->device());
    if(p != impl->deviceOverwriteItemMap.end()){
        if(p->second == item){
            impl->deviceOverwriteItemMap.erase(p);
        }
    }
}


void BodyOverwriteAddon::removeOverwriteItems(bool doClearOverwrites)
{
    vector<BodyElementOverwriteItemPtr> overwriteItems;
    overwriteItems.reserve(
        impl->linkOverwriteItemMap.size() + impl->deviceOverwriteItemMap.size());

    impl->bodyItem->traverse(
        [&overwriteItems](Item* item){
            bool doContinue = true;
            if(auto overwriteItem = dynamic_cast<BodyElementOverwriteItem*>(item)){
                overwriteItems.push_back(overwriteItem);
            } else if(auto bodyItem = dynamic_cast<BodyItem*>(item)){
                doContinue = false;
            }
            return doContinue;
        },
        false);

    // note 1: The overwrite items must be removed using the overwriteItems array
    // instead of removing them directly in the above for loop to avoid crashes.
    // note 2: The item must be processed in the reverse order to leave all the
    // links and devices as they are
    for(auto it = overwriteItems.rbegin(); it != overwriteItems.rend(); ++it){
        auto& item = *it;
        if(!doClearOverwrites){
            item->releaseOverwriteTarget(); // leave the device as it is
        }
        item->removeFromParentItem();
    }
}
