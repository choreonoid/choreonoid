/**
   @author Shin'ichiro Nakaoka
*/

#include "SubmersibleSimulatorItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

class SubmersibleSamplePlugin : public Plugin
{
public:
    
    SubmersibleSamplePlugin() : Plugin("SubmersibleSample") {
        require("Body");
    }
    
    virtual bool initialize() {
        SubmersibleSimulatorItem::initializeClass(this);
        return true;
    }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(SubmersibleSamplePlugin);
