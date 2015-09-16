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
#include <boost/bind.hpp>

#include <iostream>

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

std::vector<Vector3, Eigen::aligned_allocator<Vector3> > resistancePoints;

}


void SubmersibleSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<SubmersibleSimulatorItem>("SubmersibleSimulatorItem");
    im.addCreationPanel<SubmersibleSimulatorItem>();

    resistancePoints.push_back(Vector3( 0.8,  0.5,  0.4));
    resistancePoints.push_back(Vector3( 0.8, -0.5,  0.4));
    resistancePoints.push_back(Vector3( 0.8, -0.5, -0.4));
    resistancePoints.push_back(Vector3( 0.8,  0.5, -0.4));
    resistancePoints.push_back(Vector3(-0.8,  0.5,  0.4));
    resistancePoints.push_back(Vector3(-0.8, -0.5,  0.4));
    resistancePoints.push_back(Vector3(-0.8, -0.5, -0.4));
    resistancePoints.push_back(Vector3(-0.8,  0.5, -0.4));
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
