#ifndef CNOID_PCL_PLUGIN_PCL_PLUGIN_H
#define CNOID_PCL_PLUGIN_PCL_PLUGIN_H

#include <cnoid/Plugin>

namespace cnoid {

class PCLPlugin : public Plugin
{
public:
    
    PCLPlugin();

    static PCLPlugin* instance();
    
    virtual bool initialize() override;
};

}

#endif

