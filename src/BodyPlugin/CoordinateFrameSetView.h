#ifndef CNOID_BODY_PLUGIN_COORDINATE_FRAME_SET_VIEW_H
#define CNOID_BODY_PLUGIN_COORDINATE_FRAME_SET_VIEW_H

#include <cnoid/View>

namespace cnoid {

class CoordinateFrameSetView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    CoordinateFrameSetView();
    virtual ~CoordinateFrameSetView();
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

    class Impl;
    
private:
    Impl* impl;
};

}

#endif
