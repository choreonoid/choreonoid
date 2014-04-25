/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_JOINT_SLIDER_VIEW_H_INCLUDED
#define CNOID_BODYPLUGIN_JOINT_SLIDER_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

class JointSliderViewImpl;
        
class JointSliderView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    JointSliderView();
    virtual ~JointSliderView();
            
private:
    JointSliderViewImpl* impl;

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
};
}

#endif
