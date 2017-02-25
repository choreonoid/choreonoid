/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>

using namespace cnoid;

class SceneEffectsPlugin : public Plugin
{
public:
    
    SceneEffectsPlugin() : Plugin("SceneEffects") {
        require("Body");
    }
    
    virtual bool initialize() {
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SceneEffectsPlugin);
