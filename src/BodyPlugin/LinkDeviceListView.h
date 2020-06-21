#ifndef CNOID_BODY_PLUGIN_LINK_DEVICE_LIST_VIEW_H
#define CNOID_BODY_PLUGIN_LINK_DEVICE_LIST_VIEW_H

#include <cnoid/View>

namespace cnoid {

class BodyItem;

class LinkDeviceListView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    LinkDeviceListView();
    virtual ~LinkDeviceListView();

    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;
            
private:
    class Impl;
    Impl* impl;
};

}

#endif
