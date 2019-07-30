/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_JOINT_SLIDER_VIEW_H
#define CNOID_BODYPLUGIN_JOINT_SLIDER_VIEW_H

#include <cnoid/View>

namespace cnoid {

class JointSliderViewImpl;
        
class JointSliderView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    JointSliderView();
    virtual ~JointSliderView();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
            
private:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

    JointSliderViewImpl* impl;
};

}

#endif
