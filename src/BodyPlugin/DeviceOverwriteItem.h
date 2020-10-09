#ifndef CNOID_BODY_PLUGIN_DEVICE_OVERWRITE_ITEM_H
#define CNOID_BODY_PLUGIN_DEVICE_OVERWRITE_ITEM_H

#include "BodyElementOverwriteItem.h"
#include <cnoid/LocatableItem>
#include <cnoid/RenderableItem>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class Body;
class Link;
class Device;
class DeviceOverwriteItem;

class CNOID_EXPORT DeviceOverwriteMediator : public Referenced
{
public:
    static void registerMediator(const std::string& deviceTypeName, DeviceOverwriteMediator* mediator);
    static DeviceOverwriteMediator* findMediator(const std::string& id, const std::string& deviceTypeName);
    
    const std::string& id() { return id_; }
    virtual Device* findDevice(Body* body, const Mapping* info);
    virtual Device* restoreDevice(Body* body, Device* device = nullptr, const Mapping* info = nullptr) = 0;
    virtual bool storeInformation(DeviceOverwriteItem* item, Mapping* info) = 0;
    
    class DeviceInfo {
    public:
        DeviceInfo(Device* device, DeviceOverwriteItem* item, bool isNewDevice, bool isNewItem)
            : device_(device), item_(item), isNewDevice_(isNewDevice), isNewItem_(isNewItem) { }
        explicit operator bool() const { return device_ != nullptr; }
        Device* device() { return device_; }
        template<class DeviceType> DeviceType* device() { return dynamic_cast<DeviceType*>(device_); }
        DeviceOverwriteItem* item() { return item_; }
        bool isNewDevice() const { return isNewDevice_; }
        bool isNewItem() const { return isNewItem_; }

    private:
        Device* device_;
        DeviceOverwriteItem* item_;
        bool isNewDevice_;
        bool isNewItem_;
    };
    
    DeviceInfo findOrCreateDevice(BodyItem* bodyItem,  bool doCreateOverwriteItem = false);
    
protected:
    DeviceOverwriteMediator(const std::string& id);
    static bool restoreDeviceName(Device* device, const Mapping* info);
    static bool restoreDeviceLink(Device* device, const Mapping* info, Body* body);
    
private:
    std::string id_;
};

typedef ref_ptr<DeviceOverwriteMediator> DeviceOverwriteMediatorPtr;


class CNOID_EXPORT DeviceOverwriteItem : public BodyElementOverwriteItem,
                                         public LocatableItem, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    DeviceOverwriteItem();
    DeviceOverwriteItem(const DeviceOverwriteItem& org);
    virtual ~DeviceOverwriteItem();

    virtual bool setName(const std::string& name) override;

    bool setDevice(BodyItem* bodyItem, Device* device, bool isAdditionalDevice);
    Device* device();
    template<class DeviceType> DeviceType* device() {
        return dynamic_cast<DeviceType*>(device());
    }
    bool isAdditionalDevice() const;
    bool isPreExistingDevice() const;

    // This function returns the original device before overwriting if it is
    // originally existing one.
    Device* originalDevice();

    bool restoreOriginalDevice();
    
    void setMediatorId(const std::string& id);

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;

    // RenderableItem function. This returns the coordinate marker.
    virtual SgNode* getScene() override;

    class Impl;

protected:
    virtual Item* doDuplicate(Item* duplicatedParentItem) const override;
    virtual void onDisconnectedFromBodyItem() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    Impl* impl;
};

typedef ref_ptr<DeviceOverwriteItem> DeviceOverwriteItemPtr;

}

#endif
