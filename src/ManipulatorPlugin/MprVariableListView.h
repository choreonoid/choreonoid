#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_LIST_VIEW_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_VARIABLE_LIST_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprVariableListView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    MprVariableListView();
    virtual ~MprVariableListView();

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
