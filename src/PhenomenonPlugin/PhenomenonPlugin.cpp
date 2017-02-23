/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>

using namespace cnoid;

class PhenomenonPlugin : public Plugin
{
public:
    
    PhenomenonPlugin() : Plugin("Phenomenon") {
        require("Body");
    }
    
    virtual bool initialize() {
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(PhenomenonPlugin);
