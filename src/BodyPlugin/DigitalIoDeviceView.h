#ifndef CNOID_BODY_PLUGIN_DIGITAL_IO_DEVICE_VIEW_H
#define CNOID_BODY_PLUGIN_DIGITAL_IO_DEVICE_VIEW_H

#include <cnoid/View>

namespace cnoid {

class DigitalIoDeviceView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    DigitalIoDeviceView();
    ~DigitalIoDeviceView();

    class Impl;

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    Impl* impl;
};

}

#endif
