#include "IoConnectionMapItem.h"
#include "WorldItem.h"
#include "BodyWorldAddon.h"
#include "BodyItem.h"
#include <cnoid/IoConnectionMap>
#include <cnoid/DigitalIoDevice>
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

    Impl(IoConnectionMapItem* self);
    Impl(IoConnectionMapItem* self, const Impl& org);
    void updateIoDeviceInstances(bool enableWarningMessages);
    void forEachIoDevice(
        WorldItem* worldItem, std::function<void(BodyItem* bodyItem, DigitalIoDevice* device)> callback) const;
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


void IoConnectionMapItem::forEachIoDevice
(std::function<void(BodyItem* bodyItem, DigitalIoDevice* device)> callback) const
{
    if(auto worldItem = findOwnerItem<WorldItem>()){
        impl->forEachIoDevice(worldItem, callback);
    }
}


void IoConnectionMapItem::Impl::forEachIoDevice
(WorldItem* worldItem, std::function<void(BodyItem* bodyItem, DigitalIoDevice* device)> callback) const
{
    for(auto& bodyItem : worldItem->descendantItems<BodyItem>()){
        for(auto& device : bodyItem->body()->devices<DigitalIoDevice>()){
            callback(bodyItem, device);
        }
    }
}


void IoConnectionMapItem::updateIoDeviceInstances(bool enableWarningMessages)
{
    impl->updateIoDeviceInstances(enableWarningMessages);
}


void IoConnectionMapItem::Impl::updateIoDeviceInstances(bool enableWarningMessages)
{
    typedef unordered_map<string, DigitalIoDevice*> IoDeviceMap;
    typedef unordered_map<string, IoDeviceMap> IoDeviceMapMap;
    IoDeviceMapMap allIoDeviceMap;
    unordered_set<DigitalIoDevice*> availableDevices;

    if(auto worldItem = self->findOwnerItem<WorldItem>()){
        forEachIoDevice(
            worldItem, 
            [&](BodyItem* bodyItem, DigitalIoDevice* device){
                allIoDeviceMap[bodyItem->name()][device->name()] = device;
                availableDevices.insert(device);
            });
    }

    for(auto& connection : *connectionMap){
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
    return impl->connectionMap->read(archive);
}
