/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "RobotAccessItem.h"

using namespace cnoid;

RobotAccessItem::RobotAccessItem()
{

}


RobotAccessItem::RobotAccessItem(const RobotAccessItem& org)
    : Item(org)
{
    
}


RobotAccessItem::~RobotAccessItem()
{

}


bool RobotAccessItem::connectToRobot()
{
    return true;
}


bool RobotAccessItem::disconnectFromRobot()
{
    return true;
}


bool RobotAccessItem::activateServos(bool on)
{
    return true;
}


bool RobotAccessItem::setStateReadingEnabled(bool on)
{
    return false;
}


bool RobotAccessItem::sendCurrentPose()
{
    return false;
}


bool RobotAccessItem::setPlaybackSyncEnabled(bool on)
{
    return false;
}


void RobotAccessItem::doPutProperties(PutPropertyFunction& putProperty)
{

}


bool RobotAccessItem::store(Archive& archive)
{
    return true;
}


bool RobotAccessItem::restore(const Archive& archive)
{
    return true;
}

