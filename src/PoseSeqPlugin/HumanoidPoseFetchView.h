#ifndef CNOID_POSE_SEQ_PLUGIN_HUMANOID_POSE_FETCH_VIEW_H
#define CNOID_POSE_SEQ_PLUGIN_HUMANOID_POSE_FETCH_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT HumanoidPoseFetchView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    HumanoidPoseFetchView();
    ~HumanoidPoseFetchView();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;
        
private:
    class Impl;
    Impl* impl;
};

}

#endif
