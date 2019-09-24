#include "SignalIoConnectionMapItem.h"
#include <cnoid/SignalIoConnectionMap>
#include <cnoid/SignalIoDevice>
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

class SignalIoConnectionMapItem::Impl
{
public:
    SignalIoConnectionMapItem* self;
    SignalIoConnectionMapPtr connectionMap;
    weak_ref_ptr<Item> lastOwnerItem;

    Impl(SignalIoConnectionMapItem* self);
    Impl(SignalIoConnectionMapItem* self, const Impl& org);
    Item* findOwnerItem();
};

}


void SignalIoConnectionMapItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<SignalIoConnectionMapItem>(N_("SignalIoConnectionMapItem"));
    im.addCreationPanel<SignalIoConnectionMapItem>();
}


SignalIoConnectionMapItem::SignalIoConnectionMapItem()
{
    impl = new Impl(this);
}


SignalIoConnectionMapItem::Impl::Impl(SignalIoConnectionMapItem* self)
    : self(self)
{
    connectionMap = new SignalIoConnectionMap;
}


SignalIoConnectionMapItem::SignalIoConnectionMapItem(const SignalIoConnectionMapItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


SignalIoConnectionMapItem::Impl::Impl(SignalIoConnectionMapItem* self, const Impl& org)
    : self(self)
{
    connectionMap = new SignalIoConnectionMap(*org.connectionMap);
}


SignalIoConnectionMapItem::~SignalIoConnectionMapItem()
{
    delete impl;
}


Item* SignalIoConnectionMapItem::doDuplicate() const
{
    return new SignalIoConnectionMapItem(*this);
}


SignalIoConnectionMap* SignalIoConnectionMapItem::connectionMap()
{
    return impl->connectionMap;
}


const SignalIoConnectionMap* SignalIoConnectionMapItem::connectionMap() const
{
    return impl->connectionMap;
}


Item* SignalIoConnectionMapItem::Impl::findOwnerItem()
{
    Item* ownerItem = self->findOwnerItem<WorldItem>();
    if(!ownerItem){
        ownerItem = self->getLocalRootItem();
    }
    return ownerItem;
}


void SignalIoConnectionMapItem::onPositionChanged()
{
    auto item = impl->lastOwnerItem.lock();
    if(!item || item != impl->findOwnerItem()){
        refreshIoDeviceInstances(true);
    }
}


void SignalIoConnectionMapItem::forEachIoDevice
(std::function<void(BodyItem* bodyItem, SignalIoDevice* device)> callback) const
{
    auto ownerItem = impl->findOwnerItem();
    ItemList<BodyItem> bodyItems;
    bodyItems.extractSubTreeItems(ownerItem);
    for(auto& bodyItem : bodyItems){
        for(auto& device : bodyItem->body()->devices<SignalIoDevice>()){
            callback(bodyItem, device);
        }
    }

    impl->lastOwnerItem = ownerItem;
}


void SignalIoConnectionMapItem::refreshIoDeviceInstances(bool enableWarningMessages)
{
    typedef unordered_map<string, SignalIoDevice*> IoDeviceMap;
    typedef unordered_map<string, IoDeviceMap> IoDeviceMapMap;
    IoDeviceMapMap allIoDeviceMap;
    unordered_set<SignalIoDevice*> availableDevices;

    forEachIoDevice(
        [&](BodyItem* bodyItem, SignalIoDevice* device){
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


bool SignalIoConnectionMapItem::store(Archive& archive)
{
    return impl->connectionMap->write(archive);
}


bool SignalIoConnectionMapItem::restore(const Archive& archive)
{
    if(impl->connectionMap->read(archive)){
        archive.addPostProcess([&](){ refreshIoDeviceInstances(true);});
        return true;
    }
    return false;
}
