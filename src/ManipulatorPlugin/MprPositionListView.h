#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_VIEW_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprPositionListView : public View
{
public:
    enum BodySyncMode { NoBodySync, DirectBodySync, TwoStageBodySync };
    
    // For the application customization
    static void setDefaultBodySyncMode(BodySyncMode mode);
    
    static void initializeClass(ExtensionManager* ext);

    MprPositionListView();
    virtual ~MprPositionListView();

    void setBodySyncMode(BodySyncMode mode);

protected:
    virtual void onAttachedMenuRequest(MenuManager& menuManager) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
