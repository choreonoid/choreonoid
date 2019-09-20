#ifndef CNOID_BODY_PLUGIN_SIGNAL_IO_CONNECTION_VIEW_H
#define CNOID_BODY_PLUGIN_SIGNAL_IO_CONNECTION_VIEW_H

#include <cnoid/View>

namespace cnoid {

class SignalIoConnectionView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    SignalIoConnectionView();
    virtual ~SignalIoConnectionView();
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
