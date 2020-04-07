#ifndef CNOID_BODY_PLUGIN_JOINT_DISPLACEMENT_VIEW_H
#define CNOID_BODY_PLUGIN_JOINT_DISPLACEMENT_VIEW_H

#include <cnoid/View>

namespace cnoid {

class JointDisplacementView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    JointDisplacementView();
    virtual ~JointDisplacementView();

    class Impl;
    
protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual void onAttachedMenuRequest(MenuManager& menuManager) override;
    
private:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

    Impl* impl;
};

}

#endif
