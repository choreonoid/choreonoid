#include "SampleCameraEffectSimulatorItem.h"
#include <cnoid/Archive>
#include <cnoid/Body>
#include <cnoid/Camera>
#include <cnoid/ExtensionManager>
#include <cnoid/ItemManager>
#include <cnoid/Joystick>
#include <cnoid/SimulatorItem>

using namespace std;
using namespace cnoid;

namespace {

std::shared_ptr<Joystick> joystick;

}

void SampleCameraEffectSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<SampleCameraEffectSimulatorItem, SubSimulatorItem>("SampleCameraEffectSimulatorItem")
        .addCreationPanel<SampleCameraEffectSimulatorItem>();
}


SampleCameraEffectSimulatorItem::SampleCameraEffectSimulatorItem()
    : SubSimulatorItem()
{
    cameras.clear();
}


SampleCameraEffectSimulatorItem::SampleCameraEffectSimulatorItem(const SampleCameraEffectSimulatorItem& org)
    : SubSimulatorItem(org)
{
    cameras.clear();
}


SampleCameraEffectSimulatorItem::~SampleCameraEffectSimulatorItem()
{

}


bool SampleCameraEffectSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    cameras.clear();

    const vector<SimulationBody*>& simBodies = simulatorItem->simulationBodies();
    for(auto& simBody : simBodies) {
        Body* body = simBody->body();
        cameras << body->devices();
    }

    if(cameras.size()) {
        simulatorItem->addPreDynamicsFunction([&](){ onPreDynamicsFunction(); });
        joystick.reset(new Joystick);
    }

    return true;
}


void SampleCameraEffectSimulatorItem::onPreDynamicsFunction()
{
    joystick->readCurrentState();

    for(auto camera : cameras) {
        bool stateChanged = false;
        double salt_amount = camera->info()->get("salt_amount", 0.0);
        if(joystick->getButtonState(Joystick::X_BUTTON)) {
            camera->info()->write("salt_amount", std::max(0.0, salt_amount - 0.001));
            stateChanged = true;
        } else if(joystick->getButtonState(Joystick::Y_BUTTON)) {
            camera->info()->write("salt_amount", std::min(1.0, salt_amount + 0.001));
            stateChanged = true;
        }
        if(stateChanged) {
            camera->notifyInfoChange();
        }
    }
}


Item* SampleCameraEffectSimulatorItem::doDuplicate() const
{
    return new SampleCameraEffectSimulatorItem(*this);
}


void SampleCameraEffectSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);
}


bool SampleCameraEffectSimulatorItem::store(Archive& archive)
{
    if(!SubSimulatorItem::store(archive)) {
        return false;
    }
    return true;
}


bool SampleCameraEffectSimulatorItem::restore(const Archive& archive)
{
    if(!SubSimulatorItem::restore(archive)) {
        return false;
    }
    return true;
}