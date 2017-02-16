/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include "SceneFountain.h"

using namespace cnoid;

class PhenomenonPlugin : public Plugin
{
public:
    
    PhenomenonPlugin() : Plugin("Phenomenon") {
        require("Body");
    }
    
    virtual bool initialize() {

        SceneFountain::initializeClass();

        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(PhenomenonPlugin);
