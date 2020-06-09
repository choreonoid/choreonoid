#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_MULTI_VARIABLE_LIST_VIEW_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_MULTI_VARIABLE_LIST_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprMultiVariableListView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    MprMultiVariableListView();
    virtual ~MprMultiVariableListView();

    class Impl;

protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    Impl* impl;
};

}

#endif
