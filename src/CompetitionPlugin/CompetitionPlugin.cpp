/**
   @author Shin'ichiro Nakaoka
*/

#include "CompetitorMarker.h"
#include <cnoid/Plugin>

using namespace cnoid;

class CompetitionPlugin : public Plugin
{
public:
    CompetitionPlugin() : Plugin("Competition") {
        require("Body");
    }
    
    virtual bool initialize() {
        return true;
    }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(CompetitionPlugin);
