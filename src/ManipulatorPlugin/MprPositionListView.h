#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_VIEW_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprPositionListView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    MprPositionListView();
    virtual ~MprPositionListView();

    enum BodySyncMode { NoBodySync, DirectBodySync, TwoStageBodySync };
    void setBodySyncMode(BodySyncMode mode);

    class Impl;
    
protected:
    virtual void onAttachedMenuRequest(MenuManager& menuManager) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    Impl* impl;
};

}

#endif
