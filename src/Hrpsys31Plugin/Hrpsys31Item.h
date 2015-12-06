/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_HRPSYS31_PLUGIN_HRPSYS31_ITEM_H_INCLUDED
#define CNOID_HRPSYS31_PLUGIN_HRPSYS31_ITEM_H_INCLUDED

#include <cnoid/RobotAccessItem>
#include "exportdecl.h"

namespace cnoid {

class Hrpsys31ItemImpl;

class CNOID_EXPORT Hrpsys31Item : public RobotAccessItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    Hrpsys31Item();
    Hrpsys31Item(const Hrpsys31Item& org);

    virtual bool connectToRobot();
    virtual bool disconnectFromRobot();
    virtual bool activateServos(bool on);
    virtual bool setStateReadingEnabled(bool on);
    virtual bool sendCurrentPose();
    virtual bool setPlaybackSyncEnabled(bool on);
        
protected:
    virtual ~Hrpsys31Item();
        
    virtual void onPositionChanged();
    virtual void onDisconnectedFromRoot();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
        
private:
    Hrpsys31ItemImpl* impl;
};
    
typedef ref_ptr<Hrpsys31Item> Hrpsys31ItemPtr;
}

#endif
