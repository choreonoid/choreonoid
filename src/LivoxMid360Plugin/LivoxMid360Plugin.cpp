#include "LivoxMid360.h"
#include "GLLivoxMid360Simulator.h"
#include <cnoid/GLVisionSimulatorItem>
#include <cnoid/Plugin>

using namespace std;
using namespace cnoid;

namespace {

class LivoxMid360Plugin : public Plugin
{
public:
    LivoxMid360Plugin();
    virtual bool initialize() override;
    virtual bool finalize() override;
};

}


LivoxMid360Plugin::LivoxMid360Plugin()
    : Plugin("LivoxMid360")
{
    require("GLVisionSimulator");
}


bool LivoxMid360Plugin::initialize()
{
    GLVisionSensorSimulator::registerSimulator<LivoxMid360>(
        [](LivoxMid360* sensor){ return new GLLivoxMid360Simulator(sensor); });

    return true;
}


bool LivoxMid360Plugin::finalize()
{
    GLVisionSensorSimulator::unregisterSimulator<LivoxMid360>();
    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(LivoxMid360Plugin)
