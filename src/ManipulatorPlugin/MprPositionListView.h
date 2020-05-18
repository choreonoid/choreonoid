#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_VIEW_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_VIEW_H

#include <cnoid/View>

namespace cnoid {

class MprPositionListView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    MprPositionListView();
    virtual ~MprPositionListView();

    class Impl;
    
protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    Impl* impl;
};

}

#endif
