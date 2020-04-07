#ifndef CNOID_BODY_PLUGIN_LINK_POSITION_VIEW_H
#define CNOID_BODY_PLUGIN_LINK_POSITION_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkPositionView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static LinkPositionView* instance();

    LinkPositionView();
    virtual ~LinkPositionView();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual void onAttachedMenuRequest(MenuManager& menuManager) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
