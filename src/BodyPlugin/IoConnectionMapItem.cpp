#include "IoConnectionMapItem.h"
#include <cnoid/IoConnectionMap>
#include <cnoid/DigitalIoDevice>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <unordered_map>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class IoConnectionMapItem::Impl
{
public:
    IoConnectionMapItem* self;
    IoConnectionMapPtr connectionMap;
    weak_ref_ptr<Item> lastOwnerItem;

    Impl(IoConnectionMapItem* self);
    Impl(IoConnectionMapItem* self, const Impl& org);
    Item* findOwnerItem();
};

}


void IoConnectionMapItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<IoConnectionMapItem>(N_("IoConnectionMapItem"));
    im.addCreationPanel<IoConnectionMapItem>();
}


IoConnectionMapItem::IoConnectionMapItem()
{
    impl = new Impl(this);
}


IoConnectionMapItem::Impl::Impl(IoConnectionMapItem* self)
    : self(self)
{
    connectionMap = new IoConnectionMap;
}


IoConnectionMapItem::IoConnectionMapItem(const IoConnectionMapItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


IoConnectionMapItem::Impl::Impl(IoConnectionMapItem* self, const Impl& org)
    : self(self)
{
    connectionMap = new IoConnectionMap(*org.connectionMap);
}


IoConnectionMapItem::~IoConnectionMapItem()
{
    delete impl;
}


Item* IoConnectionMapItem::doDuplicate() const
{
    return new IoConnectionMapItem(*this);
}


IoConnectionMap* IoConnectionMapItem::connectionMap()
{
    return impl->connectionMap;
}


const IoConnectionMap* IoConnectionMapItem::connectionMap() const
{
    return impl->connectionMap;
}


Item* IoConnectionMapItem::Impl::findOwnerItem()
{
    Item* ownerItem = self->findOwnerItem<WorldItem>();
    if(!ownerItem){
        ownerItem = self->getLocalRootItem();
    }
    return ownerItem;
}


void IoConnectionMapItem::onPositionChanged()
{
    auto item = impl->lastOwnerItem.lock();
    if(!item || item != impl->findOwnerItem()){
        refreshIoDeviceInstances(true);
    }
}


void IoConnectionMapItem::forEachIoDevice
(std::function<void(BodyItem* bodyItem, DigitalIoDevice* device)> callback) const
{
    auto ownerItem = impl->findOwnerItem();
    ItemList<BodyItem> bodyItems;
    bodyItems.extractSubTreeItems(ownerItem);
    for(auto& bodyItem : bodyItems){
        for(auto& device : bodyItem->body()->devices<DigitalIoDevice>()){
            callback(bodyItem, device);
        }
    }

    impl->lastOwnerItem = ownerItem;
}


void IoConnectionMapItem::refreshIoDeviceInstances(bool enableWarningMessages)
{
    typedef unordered_map<string, DigitalIoDevice*> IoDeviceMap;
    typedef unordered_map<string, IoDeviceMap> IoDeviceMapMap;
    IoDeviceMapMap allIoDeviceMap;
    unordered_set<DigitalIoDevice*> availableDevices;

    forEachIoDevice(
        [&](BodyItem* bodyItem, DigitalIoDevice* device){
            allIoDeviceMap[bodyItem->name()][device->name()] = device;
            availableDevices.insert(device);
        });

    for(auto& connection : *impl->connectionMap){
        for(int i=0; i < 2; ++i){
            auto device = connection->device(i);
            if(!device || availableDevices.find(device) == availableDevices.end()){
                bool found = false;
                auto p = allIoDeviceMap.find(connection->bodyName(i));
                if(p != allIoDeviceMap.end()){
                    auto& ioDeviceMap = p->second;
                    auto q = ioDeviceMap.find(connection->deviceName(i));
                    if(q != ioDeviceMap.end()){
                        auto correspondingDevice = q->second;
                        connection->setDevice(i, correspondingDevice);
                        found = true;
                    }
                }
                if(!found){
                    connection->setDevice(i, nullptr);
                }
            }
        }
    }
}


bool IoConnectionMapItem::store(Archive& archive)
{
    return impl->connectionMap->write(archive);
}


bool IoConnectionMapItem::restore(const Archive& archive)
{
    if(impl->connectionMap->read(archive)){
        archive.addPostProcess([&](){ refreshIoDeviceInstances(true);});
        return true;
    }
    return false;
}
