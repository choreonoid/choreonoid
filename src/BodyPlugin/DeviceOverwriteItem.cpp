#include "DeviceOverwriteItem.h"
#include "BodyOverwriteAddon.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Device>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <cnoid/MessageView>
#include <cnoid/PositionDragger>
#include <fmt/format.h>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

typedef std::unordered_map<string, DeviceOverwriteMediatorPtr> MediatorMap;
std::unordered_map<string, MediatorMap> idToMediatorMap;

class DeviceLocation : public LocationProxy
{
public:
    DeviceOverwriteItem::Impl* impl;

    DeviceLocation(DeviceOverwriteItem::Impl* impl);
    virtual Isometry3 getLocation() const override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual Item* getCorrespondingItem() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

}

namespace cnoid {

class DeviceOverwriteItem::Impl
{
public:
    DeviceOverwriteItem* self;
    DevicePtr device;
    bool isAdditionalDevice;
    int additionalDeviceLinkIndex;
    DevicePtr originalDevice;
    string originalDeviceName;
    string mediatorId;
    ref_ptr<DeviceLocation> deviceLocation;
    LocationProxyPtr linkLocation;
    SgPosTransformPtr linkPosTransform;
    PositionDraggerPtr deviceOffsetMarker;
    ScopedConnection bodyItemConnection;
    ScopedConnection deviceConnection;
    
    Impl(DeviceOverwriteItem* self);
    Impl(DeviceOverwriteItem* self, const Impl& org);
    void clear();
    void clearLocationProxies();
    bool setDevice(BodyItem* bodyItem, Device* device, Device* originalDevice, bool isDuplicated);
    bool restoreOriginalDevice();
    void updateDeviceOffsetMarker();
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


void DeviceOverwriteMediator::registerMediator
(const std::string& deviceTypeName, DeviceOverwriteMediator* mediator)
{
    idToMediatorMap[mediator->id()][deviceTypeName] = mediator;
}


DeviceOverwriteMediator* DeviceOverwriteMediator::findMediator
(const std::string& id, const std::string& deviceTypeName)
{
    auto p = idToMediatorMap.find(id);
    if(p != idToMediatorMap.end()){
        auto& mediatorMap = p->second;
        auto q = mediatorMap.find(deviceTypeName);
        if(q != mediatorMap.end()){
            return q->second;
        }
    }
    return nullptr;
}


DeviceOverwriteMediator::DeviceOverwriteMediator(const std::string& id)
    : id_(id)
{

}


Device* DeviceOverwriteMediator::findDevice(Body* body, const Mapping* info)
{
    if(!info){
        return nullptr;
    }
    string type;
    if(!info->read("device_type", type)){
        return nullptr;
    }
    string name;
    if(!info->read("original_device_name", name)){
        info->read("device_name", name);
    }
    string linkName;
    info->read("link_name", linkName);

    int maxScore = 0;
    Device* found = nullptr;
    auto devices = body->devices();
    for(auto& device : devices){
        if(device->typeName() == type){
            int score = 1;
            if(device->name() == name){
                score++;
            }
            if(device->link()->name() == linkName){
                score++;
            }
            if(score > maxScore){
                found = device;
                maxScore = score;
            }
        }
    }
    return found;
}


DeviceOverwriteMediator::DeviceInfo DeviceOverwriteMediator::findOrCreateDevice
(BodyItem* bodyItem, bool doCreateOverwriteItem)
{
    DeviceOverwriteItemPtr overwriteItem;
    auto body = bodyItem->body();
    DevicePtr device = findDevice(body, nullptr);
    bool deviceFound;
    if(device){
        deviceFound = true;
        overwriteItem = bodyItem->getAddon<BodyOverwriteAddon>()->findDeviceOverwriteItem(device);
    } else {
        deviceFound = false;
        device = restoreDevice(body);
        if(!doCreateOverwriteItem){
            bodyItem->body()->addDevice(device, device->link());
        }
    }
    if(overwriteItem || !doCreateOverwriteItem){
        return DeviceInfo(device, overwriteItem, !deviceFound, false);
    }
    bool added = false;
    overwriteItem = new DeviceOverwriteItem;
    if(!device->name().empty()){
        overwriteItem->setName(device->name());
    } else {
        overwriteItem->setName(device->typeName());
    }
    overwriteItem->setMediatorId(id_);
    if(overwriteItem->setDevice(bodyItem, device, !deviceFound)){
        added = bodyItem->addChildItem(overwriteItem);
    }
    if(!added){
        overwriteItem.reset();
    }
    return DeviceInfo(device, overwriteItem, !deviceFound, true);
}


bool DeviceOverwriteMediator::restoreDeviceName(Device* device, const Mapping* info)
{
    string name;
    if(info->read("device_name", name)){
        device->setName(name);
        return true;
    }
    return false;
}


bool DeviceOverwriteMediator::restoreDeviceLink(Device* device, const Mapping* info, Body* body)
{
    string name;
    if(info->read("link_name", name)){
        if(auto link = body->link(name)){
            device->setLink(link);
            return true;
        }
    }
    return false;
}


void DeviceOverwriteItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<DeviceOverwriteItem, BodyElementOverwriteItem>(N_("DeviceOverwriteItem"));
    im.addAlias<DeviceOverwriteItem>("DeviceOverwriteItem", "BodyEdit");
}


DeviceOverwriteItem::DeviceOverwriteItem()
{
    impl = new Impl(this);
    setAttribute(Item::Attached);
}


DeviceOverwriteItem::Impl::Impl(DeviceOverwriteItem* self)
    : self(self)
{
    isAdditionalDevice = false;
}


DeviceOverwriteItem::DeviceOverwriteItem(const DeviceOverwriteItem& org)
    : BodyElementOverwriteItem(org)
{
    impl = new Impl(this, *org.impl);
}


DeviceOverwriteItem::Impl::Impl(DeviceOverwriteItem* self, const Impl& org)
    : Impl(self)
{
    mediatorId = org.mediatorId;
}


DeviceOverwriteItem::~DeviceOverwriteItem()
{
    delete impl;
}


bool DeviceOverwriteItem::setName(const std::string& name)
{
    if(impl->device){
        impl->device->setName(name);
    }
    Item::setName(name);
    return true;
}


Item* DeviceOverwriteItem::doDuplicate(Item* duplicatedParentItem) const
{
    DeviceOverwriteItemPtr duplicated = new DeviceOverwriteItem(*this);

    if(auto bodyItem = dynamic_cast<BodyItem*>(duplicatedParentItem)){
        auto device = bodyItem->body()->device(impl->device->index());
        if(!duplicated->impl->setDevice(bodyItem, device, impl->originalDevice, true)){
            duplicated.reset();
        }
    }
    
    return duplicated.retn();
}


void DeviceOverwriteItem::Impl::clear()
{
    self->setBodyItem(nullptr);
    device.reset();
    isAdditionalDevice = false;
    originalDevice.reset();
    originalDeviceName.clear();
    clearLocationProxies();
}


void DeviceOverwriteItem::Impl::clearLocationProxies()
{
    if(deviceLocation){
        deviceLocation->expire();
        deviceLocation.reset();
    }
    if(linkLocation){
        linkLocation->expire();
        linkLocation.reset();
    }
}    


bool DeviceOverwriteItem::setDevice(BodyItem* bodyItem, Device* device, bool isAdditionalDevice)
{
    return impl->setDevice(bodyItem, device, isAdditionalDevice ? nullptr : device, false);
}


bool DeviceOverwriteItem::Impl::setDevice
(BodyItem* bodyItem, Device* device, Device* originalDevice, bool isDuplicated)
{
    auto link = device->link();
    if(!link){
        return false;
    }
    auto body = bodyItem->body();
    if(link->body() != body){
        return false;
    }

    self->setBodyItem(bodyItem);
    this->device = device;
    isAdditionalDevice = !originalDevice;
    if(originalDevice){
        this->originalDevice = originalDevice->clone();
        originalDeviceName = originalDevice->name();
    } else {
        this->originalDevice.reset();
        originalDeviceName.clear();
    }
    clearLocationProxies();
        
    if(!self->bodyOverwrite()->addDeviceOverwriteItem(self)){
        clear();
        return false;
    }
    
    if(!originalDevice && !isDuplicated){
        body->addDevice(device, link);
    }

    bodyItemConnection =
        bodyItem->sigKinematicStateChanged().connect(
            [&](){ updateDeviceOffsetMarker(); });

    //! \todo Define the signal on the change of the device specification and use it here
    deviceConnection =
        device->sigStateChanged().connect(
            [&](){ updateDeviceOffsetMarker(); });

    if(deviceOffsetMarker){
        updateDeviceOffsetMarker();
    }

    return true;
}


Device* DeviceOverwriteItem::device()
{
    return impl->device;
}


bool DeviceOverwriteItem::isAdditionalDevice() const
{
    return impl->isAdditionalDevice;
}


bool DeviceOverwriteItem::isPreExistingDevice() const
{
    return !impl->isAdditionalDevice;
}


Device* DeviceOverwriteItem::originalDevice()
{
    return impl->originalDevice;
}


bool DeviceOverwriteItem::restoreOriginalDevice()
{
    return impl->restoreOriginalDevice();
}


bool DeviceOverwriteItem::Impl::restoreOriginalDevice()
{
    if(!isAdditionalDevice && originalDevice){
        return device->copyFrom(originalDevice);
    }
    return false;
}


void DeviceOverwriteItem::setMediatorId(const std::string& id)
{
    impl->mediatorId = id;
}


void DeviceOverwriteItem::onDisconnectedFromBodyItem()
{
    if(impl->isAdditionalDevice){
        bodyItem()->body()->removeDevice(impl->device);
    }
    bodyOverwrite()->removeDeviceOverwriteItem(this);
    impl->clear();
}


LocationProxyPtr DeviceOverwriteItem::getLocationProxy()
{
    if(!impl->deviceLocation){
        impl->deviceLocation = new DeviceLocation(impl);
        impl->updateDeviceOffsetMarker();
    }
    return impl->deviceLocation;
}


namespace {

DeviceLocation::DeviceLocation(DeviceOverwriteItem::Impl* impl)
    : LocationProxy(OffsetLocation),
      impl(impl)
{
    setEditable(false);
    sigAttributeChanged().connect([impl](){ impl->updateDeviceOffsetMarker(); });
}
    

Isometry3 DeviceLocation::getLocation() const
{
    if(impl->device){
        return impl->device->localPosition();
    }
    return Isometry3::Identity();
}


bool DeviceLocation::setLocation(const Isometry3& T)
{
    if(impl->device){
        impl->device->setLocalPosition(T);
    }
    //! \todo Define the signal on the change of the device specification and use it here
    impl->device->notifyStateChange();
    return true;
}


Item* DeviceLocation::getCorrespondingItem()
{
    return impl->self;
}


LocationProxyPtr DeviceLocation::getParentLocationProxy() const
{
    if(!impl->linkLocation){
        if(impl->device){
            if(auto link = impl->device->link()){
                impl->linkLocation = impl->self->bodyItem()->createLinkLocationProxy(link);
            }
        }
    }
    return impl->linkLocation;
}


SignalProxy<void()> DeviceLocation::sigLocationChanged()
{
    if(impl->device){
        //! \todo Define the signal on the change of the device specification and use it here
        return impl->device->sigStateChanged();
    }
    static Signal<void()> dummySignal;
    return dummySignal;
}

}


SgNode* DeviceOverwriteItem::getScene()
{
    if(!impl->deviceOffsetMarker){
        impl->updateDeviceOffsetMarker();
    }
    return impl->linkPosTransform;
}


void DeviceOverwriteItem::Impl::updateDeviceOffsetMarker()
{
    if(!deviceOffsetMarker){
        linkPosTransform = new SgPosTransform;
        deviceOffsetMarker = new PositionDragger(
            PositionDragger::AllAxes, PositionDragger::PositiveOnlyHandle);
        deviceOffsetMarker->setOverlayMode(true);
        deviceOffsetMarker->setDragEnabled(true);
        deviceOffsetMarker->setPixelSize(96, 3);
        deviceOffsetMarker->setDisplayMode(PositionDragger::DisplayInEditMode);

        deviceOffsetMarker->sigPositionDragged().connect(
            [&](){
                if(device){
                    device->setLocalPosition(deviceOffsetMarker->draggingPosition());
                    //! \todo Define the signal on the change of the device specification and use it here
                    device->notifyStateChange();
                }
            });
        
        linkPosTransform->addChild(deviceOffsetMarker);
    }

    if(deviceLocation){
        deviceOffsetMarker->setDragEnabled(deviceLocation->isEditable());
    }
    
    if(device){
        deviceOffsetMarker->setPosition(device->localPosition());
        linkPosTransform->setPosition(device->link()->position());
        linkPosTransform->notifyUpdate();
    }
}


void DeviceOverwriteItem::doPutProperties(PutPropertyFunction& putProperty)
{
    Item::doPutProperties(putProperty);
}


bool DeviceOverwriteItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool DeviceOverwriteItem::Impl::store(Archive& archive)
{
    bool result = false;
    if(device && device->link()){
        if(mediatorId.empty()){
            return false;
        }
        auto typeName = device->typeName();
        if(auto mediator = DeviceOverwriteMediator::findMediator(mediatorId, typeName)){
            if(mediator->storeInformation(self, &archive)){
                archive.write("mediator", mediatorId);
                archive.write("device_type", typeName);
                if(!device->name().empty()){
                    archive.write("device_name", device->name(), DOUBLE_QUOTED);
                }
                archive.write("link_name", device->link()->name(), DOUBLE_QUOTED);
                if(isAdditionalDevice){
                    archive.write("is_additional", isAdditionalDevice);
                } else {
                    if(!originalDeviceName.empty()){
                        archive.write("original_device_name", originalDeviceName, DOUBLE_QUOTED);
                    }
                }
                archive.setFloatingNumberFormat("%.9g");
                auto T = device->T_local();
                AngleAxis aa(T.linear());
                if(aa.angle() != 0.0){
                    writeDegreeAngleAxis(archive, "rotation", aa);
                }
                if(!T.translation().isZero()){
                    write(archive, "translation", T.translation());
                }
                
                result = true;
            }
        }
    }
    return result;
}


bool DeviceOverwriteItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool DeviceOverwriteItem::Impl::restore(const Archive& archive)
{
    auto bodyItem = archive.currentParentItem()->findOwnerItem<BodyItem>(true);
    if(!bodyItem){
        MessageView::instance()->putln(
            format(_("The target body item of \"{0}\" is not found."), self->name()),
            MessageView::Error);
        return false;
    }
    auto body = bodyItem->body();

    bool restored = false;
    mediatorId.clear();
    if(archive.read("mediator", mediatorId)){
        string typeName;
        if(archive.read("device_type", typeName)){
            if(auto mediator = DeviceOverwriteMediator::findMediator(mediatorId, typeName)){
                bool isAdditional = archive.get("is_additional", false);
                DevicePtr device;
                if(!isAdditional){
                    device = mediator->findDevice(body, &archive);
                }
                if(device || isAdditional){
                    device = mediator->restoreDevice(body, device, &archive);
                }
                if(device){
                    restored = self->setDevice(bodyItem, device, isAdditional);

                    if(restored){
                        Vector3 v;
                        if(cnoid::read(archive, "translation", v)){
                            device->setLocalTranslation(v);
                        }
                        AngleAxis aa;
                        if(cnoid::readDegreeAngleAxis(archive, "rotation", aa)){
                            device->setLocalRotation(aa.matrix());
                        }
                    }
                }
            }
        }
    }

    return restored;
}
