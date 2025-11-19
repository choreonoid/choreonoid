#ifndef CNOID_OMPL_PLUGIN_OMPL_PLUGIN_H
#define CNOID_OMPL_PLUGIN_OMPL_PLUGIN_H

#include <cnoid/Plugin>
#include <memory>

namespace cnoid {

class MessageOut;

class OMPLPlugin : public Plugin
{
public:
    static OMPLPlugin* instance();
    
    OMPLPlugin();
    virtual ~OMPLPlugin();
    
    virtual bool initialize() override;
    virtual bool finalize() override;

    void setMessageOutputEnabled(bool on);
    // Set the MessageOut instance to be used for OMPL log messages
    void setMessageOut(MessageOut* mout);

private:
    class Impl;
    std::unique_ptr<Impl> impl;
};

}

#endif