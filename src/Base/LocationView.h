#ifndef CNOID_BASE_LOCATION_VIEW_H
#define CNOID_BASE_LOCATION_VIEW_H

#include "View.h"

namespace cnoid {

class LocationView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    LocationView();
    ~LocationView();

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
