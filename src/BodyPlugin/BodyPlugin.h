#ifndef CNOID_BODY_PLUGIN_BODY_PLUGIN_H
#define CNOID_BODY_PLUGIN_BODY_PLUGIN_H

#include <cnoid/Plugin>

namespace cnoid {

class BodyPlugin : public Plugin
{
public:
    static BodyPlugin* instance();
    BodyPlugin();
    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual const char* description() const override;
};

}

#endif

