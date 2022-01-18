#ifndef CNOID_BODY_PLUGIN_IO_CONNECTION_VIEW_H
#define CNOID_BODY_PLUGIN_IO_CONNECTION_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT IoConnectionView : public View
{
public:
    // For the application customization
    static void setDefaultEditableIoNumberRange(int minNumber, int maxNumber);

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
