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
#include <cnoid/Joystick>
#include <boost/bind.hpp>

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

std::vector<Vector3, Eigen::aligned_allocator<Vector3> > resistancePoints;
std::vector<Vector3, Eigen::aligned_allocator<Vector3> > thrustPoints;

Joystick* joystick = 0;

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
    isEnabled_ = org.isEnabled_;
}


void SubmersibleSimulatorItem::initialize()
{
    simulatorItem = 0;
    submersible = 0;
    isEnabled_ = true;
}


SubmersibleSimulatorItem::~SubmersibleSimulatorItem()
{

}


bool SubmersibleSimulatorItem::isEnabled()
{
    return isEnabled_;
}


ItemPtr SubmersibleSimulatorItem::doDuplicate() const
{
    return new SubmersibleSimulatorItem(*this);
}


void SubmersibleSimulatorItem::onConnectedToRoot()
{
    if(!joystick){
        joystick = new Joystick();
    }
}


bool SubmersibleSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    this->simulatorItem = simulatorItem;

    submersible = 0;
    SimulationBody* simSubmersible = simulatorItem->findSimulationBody("Submersible");
    if(simSubmersible){
        submersible = simSubmersible->body();
        MessageView::instance()->putln("A submersible model has been detected.");
        simulatorItem->addPostDynamicsFunction(
            boost::bind(&SubmersibleSimulatorItem::applyResistanceForce, this));
        joystickIntervalCounter = 0;
    }
    
    return true;
}


void SubmersibleSimulatorItem::applyResistanceForce()
{
    Link* root = submersible->rootLink();

    root->F_ext().setZero();
    for(size_t i=0; i < resistancePoints.size(); ++i){
        const Vector3 a = root->R() * resistancePoints[i];
        const Vector3 v = root->v() + root->w().cross(a);
        Vector3 f = -2.0 * v;
        double l = f.norm();
        if(l > 100.0){
            f /= l;
        }
        root->f_ext() += f;
        const Vector3 p = a + root->p();
        root->tau_ext() += p.cross(f);
    }

    if(joystickIntervalCounter++ > 40){
        joystickIntervalCounter = 0;
        joystick->readCurrentState();
    }

    double thrust[2];
    thrust[0]  = -joystick->getPosition(4); // right
    thrust[1]  = -joystick->getPosition(1); // left
    for(int i=0; i < 2; ++i){
        const Vector3 f = root->R() * Vector3(15.0 * thrust[i], 0.0, 0.0);
        const Vector3 p = root->T() * thrustPoints[i];
        root->f_ext() += f;
        root->tau_ext() += p.cross(f);
    }
}
        

void SubmersibleSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty("Enabled", isEnabled_, changeProperty(isEnabled_));
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
