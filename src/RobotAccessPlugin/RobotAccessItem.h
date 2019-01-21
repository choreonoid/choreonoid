/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_ROBOT_ACCESS_PLUGIN_ROBOT_ACCESS_ITEM_H
#define CNOID_ROBOT_ACCESS_PLUGIN_ROBOT_ACCESS_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT RobotAccessItem : public Item
{
public:
    virtual bool connectToRobot();
    virtual bool disconnectFromRobot();
    virtual bool activateServos(bool on);
    virtual bool setStateReadingEnabled(bool on);
    virtual bool sendCurrentPose();
    virtual bool setPlaybackSyncEnabled(bool on);
        
protected:
    RobotAccessItem();
    RobotAccessItem(const RobotAccessItem& org);
    virtual ~RobotAccessItem();

    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
};

typedef ref_ptr<RobotAccessItem> RobotAccessItemPtr;

}

#endif
