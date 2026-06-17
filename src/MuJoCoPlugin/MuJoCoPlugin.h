#ifndef CNOID_MUJOCO_PLUGIN_MUJOCO_PLUGIN_H
#define CNOID_MUJOCO_PLUGIN_MUJOCO_PLUGIN_H

#include <cnoid/Plugin>

namespace cnoid {

class MuJoCoPlugin : public Plugin
{
public:
    MuJoCoPlugin();
    virtual bool initialize() override;
    virtual const char* description() const override;
};

}

#endif
