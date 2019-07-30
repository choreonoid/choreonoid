/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_JOINT_DISPLACEMENT_VIEW_H
#define CNOID_BODYPLUGIN_JOINT_DISPLACEMENT_VIEW_H

#include <cnoid/View>

namespace cnoid {

class JointDisplacementViewImpl;
        
class JointDisplacementView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    JointDisplacementView();
    virtual ~JointDisplacementView();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
            
private:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

    JointDisplacementViewImpl* impl;
};

}

#endif
