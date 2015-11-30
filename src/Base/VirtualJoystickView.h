/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_VIRTUAL_JOYSTICK_VIEW_H
#define CNOID_BASE_VIRTUAL_JOYSTICK_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class VirtualJoystickViewImpl;

class CNOID_EXPORT VirtualJoystickView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    VirtualJoystickView();
    ~VirtualJoystickView();

protected:
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void keyReleaseEvent(QKeyEvent* event);
    
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
    
private:
    VirtualJoystickViewImpl* impl;
};

}

#endif
