#ifndef CNOID_BASE_COORDINATE_FRAME_LIST_VIEW_H
#define CNOID_BASE_COORDINATE_FRAME_LIST_VIEW_H

#include <cnoid/View>

namespace cnoid {

class CoordinateFrameListView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    CoordinateFrameListView();
    virtual ~CoordinateFrameListView();

    class Impl;
    
protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual void onAttachedMenuRequest(MenuManager& menuManager) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    Impl* impl;
};

}

#endif
