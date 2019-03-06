/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SubmersibleSimulatorItem.h"
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Body>
#include <cnoid/Light>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

namespace {

std::vector<Vector3, Eigen::aligned_allocator<Vector3> > resistancePoints;
std::vector<Vector3, Eigen::aligned_allocator<Vector3> > thrustPoints;

std::shared_ptr<Joystick> joystick;

}


void SubmersibleSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<SubmersibleSimulatorItem>("SubmersibleSimulatorItem");
    im.addCreationPanel<SubmersibleSimulatorItem>();

    resistancePoints.push_back(Vector3( 0.3,  0.2,  0.15));
    resistancePoints.push_back(Vector3( 0.3, -0.2,  0.15));
    resistancePoints.push_back(Vector3( 0.3, -0.2, -0.15));
    resistancePoints.push_back(Vector3( 0.3,  0.2, -0.15));
    resistancePoints.push_back(Vector3(-0.3,  0.2,  0.15));
    resistancePoints.push_back(Vector3(-0.3, -0.2,  0.15));
    resistancePoints.push_back(Vector3(-0.3, -0.2, -0.15));
    resistancePoints.push_back(Vector3(-0.3,  0.2, -0.15));

    thrustPoints.push_back(Vector3(-0.3, -0.18, 0.0));
    thrustPoints.push_back(Vector3(-0.3,  0.18, 0.0));
}


SubmersibleSimulatorItem::SubmersibleSimulatorItem()
{
    initialize();
}


SubmersibleSimulatorItem::SubmersibleSimulatorItem(const SubmersibleSimulatorItem& org)
    : SubSimulatorItem(org)
{
    initialize();
}


void SubmersibleSimulatorItem::initialize()
{
    simulatorItem = 0;
    submersible = 0;
}


SubmersibleSimulatorItem::~SubmersibleSimulatorItem()
{

}


Item* SubmersibleSimulatorItem::doDuplicate() const
{
    return new SubmersibleSimulatorItem(*this);
}


bool SubmersibleSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    this->simulatorItem = simulatorItem;

    submersible = 0;
    SimulationBody* simSubmersible = simulatorItem->findSimulationBody("Submersible");
    if(simSubmersible){
        submersible = simSubmersible->body();
        light = submersible->findDevice<Light>("MainLight");
        prevLightButtonState = false;
        MessageView::instance()->putln("A submersible model has been detected.");
        simulatorItem->addPreDynamicsFunction(
            std::bind(&SubmersibleSimulatorItem::applyResistanceForce, this));
        joystick.reset(new Joystick());
        joystickIntervalCounter = 0;
    }
    
    return true;
}


void SubmersibleSimulatorItem::applyResistanceForce()
{
    Link* root = submersible->rootLink();

    // buoyancy
    Vector3 b(0, 0, submersible->mass() * 9.80665);
    root->f_ext() += b;
    Vector3 cb = root->T() * Vector3(0.0, 0.0, 0.05);
    root->tau_ext() += cb.cross(b);

    for(size_t i=0; i < resistancePoints.size(); ++i){
        Vector3 a = root->R() * resistancePoints[i];
        Vector3 v = root->v() + root->w().cross(a);
        Vector3 f = -2.0 * v;
        double l = f.norm();
        if(l > 100.0){
            f /= l;
        }
        root->f_ext() += f;
        Vector3 p = a + root->p();
        root->tau_ext() += p.cross(f);
    }

    if(joystickIntervalCounter++ > 40){
        joystickIntervalCounter = 0;
        joystick->readCurrentState();
    }

    double thrust[2];
    thrust[0]  = -joystick->getPosition(3); // right
    thrust[1]  = -joystick->getPosition(1); // left
    for(int i=0; i < 2; ++i){
        Vector3 f = root->R() * Vector3(15.0 * thrust[i], 0.0, 0.0);
        Vector3 p = root->T() * thrustPoints[i];
        root->f_ext() += f;
        root->tau_ext() += p.cross(f);
    }

    Vector3 fz(0.0, 0.0, -5.0 * joystick->getPosition(5));
    root->f_ext() += fz;
    Vector3 cz = root->T() * Vector3(0.0, 0.0, -1.5);
    root->tau_ext() += cz.cross(fz);

    if(light){
        bool lightButtonState = joystick->getButtonState(0);
        if(lightButtonState){
            if(!prevLightButtonState){
                light->on(!light->on());
                light->notifyStateChange();
            }
        }
        prevLightButtonState = lightButtonState;
    }
}
        

void SubmersibleSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);
}


bool SubmersibleSimulatorItem::store(Archive& archive)
{
    SubSimulatorItem::store(archive);
    return true;
}


bool SubmersibleSimulatorItem::restore(const Archive& archive)
{
    SubSimulatorItem::restore(archive);
    return true;
}
