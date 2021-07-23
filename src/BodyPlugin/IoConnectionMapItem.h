#ifndef CNOID_BODY_PLUGIN_IO_CONNECTION_MAP_ITEM_H
#define CNOID_BODY_PLUGIN_IO_CONNECTION_MAP_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class IoConnectionMap;
class BodyItem;
class DigitalIoDevice;

class CNOID_EXPORT IoConnectionMapItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    IoConnectionMapItem();
    IoConnectionMapItem(const IoConnectionMapItem& org);
    virtual ~IoConnectionMapItem();

    IoConnectionMap* connectionMap();
    const IoConnectionMap* connectionMap() const;

    void forEachIoDevice(std::function<void(BodyItem* bodyItem, DigitalIoDevice* device)> callback) const;
    
    void updateIoDeviceInstances(bool enableWarningMessages = true);

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<IoConnectionMapItem> IoConnectionMapItemPtr;

}

#endif

