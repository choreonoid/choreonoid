/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_ROBOT_ACCESS_PLUGIN_ROBOT_ACCESS_BAR_H_INCLUDED
#define CNOID_ROBOT_ACCESS_PLUGIN_ROBOT_ACCESS_BAR_H_INCLUDED

#include <cnoid/ToolBar>

namespace cnoid {

class RobotAccessBarImpl;
    
class RobotAccessBar : public ToolBar
{
public:
    RobotAccessBar();
    ~RobotAccessBar();

private:
    RobotAccessBarImpl* impl;
};
}

#endif
