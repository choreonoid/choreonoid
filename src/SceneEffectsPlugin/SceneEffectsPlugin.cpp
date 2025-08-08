#include <cnoid/Plugin>

using namespace cnoid;

namespace cnoid {
// Device registration functions
void registerFireDevice();
void registerFountainDevice();
void registerRainSnowDevices();
void registerSmokeDevice();

// Scene registration functions
void registerSceneFire();
void registerSceneFountain();
void registerSceneSmoke();
void registerSceneRainSnow();
}

class SceneEffectsPlugin : public Plugin
{
public:
    
    SceneEffectsPlugin() : Plugin("SceneEffects") {
        require("Body");
    }
    
    virtual bool initialize() {
        // Register device types
        registerFireDevice();
        registerFountainDevice();
        registerRainSnowDevices();
        registerSmokeDevice();

        // Register scene types and effect programs
        registerSceneFire();
        registerSceneFountain();
        registerSceneSmoke();
        registerSceneRainSnow();

        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SceneEffectsPlugin);
