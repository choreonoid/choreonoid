#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_VARIABLE_LIST_VIEW_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_VARIABLE_LIST_VIEW_BASE_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ManipulatorVariableListViewBase : public View
{
public:
    ManipulatorVariableListViewBase();
    virtual ~ManipulatorVariableListViewBase();

    class Impl;
    
protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    Impl* impl;
};

}

#endif
