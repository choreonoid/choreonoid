/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_HRPSYS31_PLUGIN_HRPSYS31_ITEM_H
#define CNOID_HRPSYS31_PLUGIN_HRPSYS31_ITEM_H

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

    virtual bool connectToRobot() override;
    virtual bool disconnectFromRobot() override;
    virtual bool activateServos(bool on) override;
    virtual bool setStateReadingEnabled(bool on) override;
    virtual bool sendCurrentPose() override;
    virtual bool setPlaybackSyncEnabled(bool on) override;
        
protected:
    virtual ~Hrpsys31Item();
        
    virtual void onPositionChanged() override;
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    Hrpsys31ItemImpl* impl;
};
    
typedef ref_ptr<Hrpsys31Item> Hrpsys31ItemPtr;

}

#endif
