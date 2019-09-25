#ifndef CNOID_BODY_PLUGIN_IO_CONNECTION_VIEW_H
#define CNOID_BODY_PLUGIN_IO_CONNECTION_VIEW_H

#include <cnoid/View>

namespace cnoid {

class IoConnectionView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    IoConnectionView();
    virtual ~IoConnectionView();
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

    class Impl;
    
private:
    Impl* impl;
};

}

#endif
