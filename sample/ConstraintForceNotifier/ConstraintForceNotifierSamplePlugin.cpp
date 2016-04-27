/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/SubSimulatorItem>
#include <cnoid/AISTSimulatorItem>
#include <cnoid/DyBody>
#include <boost/bind.hpp>
#include <sstream>

using namespace std;
using namespace cnoid;

namespace {

class ConstraintForceNotifierItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ConstraintForceNotifierItem();
    ConstraintForceNotifierItem(const ConstraintForceNotifierItem& org);

    virtual bool initializeSimulation(SimulatorItem* simulatorItem);

protected:
    virtual Item* doDuplicate() const;

private:
    void notifyContactPoints(SimulatorItem* simulator);
    void notifyBodyContactPoints(DyBody* body, ostream& os);
};


class ConstraintForceNotifierSamplePlugin : public Plugin
{
public:
    ConstraintForceNotifierSamplePlugin() : Plugin("ConstraintForceNotifierSample") {
        require("Body");
    }

    virtual bool initialize() {
        itemManager()
            .registerClass<ConstraintForceNotifierItem>("ConstraintForceNotifierItem")
            .addCreationPanel<ConstraintForceNotifierItem>();
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ConstraintForceNotifierSamplePlugin);



ConstraintForceNotifierItem::ConstraintForceNotifierItem()
{

}


ConstraintForceNotifierItem::ConstraintForceNotifierItem(const ConstraintForceNotifierItem& org)
    : SubSimulatorItem(org)
{

}


Item* ConstraintForceNotifierItem::doDuplicate() const
{
    return new ConstraintForceNotifierItem(*this);
}


bool ConstraintForceNotifierItem::initializeSimulation(SimulatorItem* simulator)
{
    AISTSimulatorItem* aistSimulator = dynamic_cast<AISTSimulatorItem*>(simulator);
    if(!aistSimulator){
        return false;
    }

    aistSimulator->setConstraintForceOutputEnabled(true);

    simulator->addPostDynamicsFunction(
        boost::bind(&ConstraintForceNotifierItem::notifyContactPoints, this, simulator));

    return true;
}


void ConstraintForceNotifierItem::notifyContactPoints(SimulatorItem* simulator)
{
    ostringstream oss;

    const vector<SimulationBody*>& simBodies = simulator->simulationBodies();
    for(size_t i=0; i < simBodies.size(); ++i){
        DyBody* body = dynamic_cast<DyBody*>(simBodies[i]->body());
        if(body){
            notifyBodyContactPoints(body, oss);
        }
    }
    string log = oss.str();
    if(!log.empty()){
        mvout() << log << endl;
    }
}


void ConstraintForceNotifierItem::notifyBodyContactPoints(DyBody* body, ostream& os)
{
    bool put = false;
    for(int i=0; i < body->numLinks(); ++i){
        DyLink* link = body->link(i);
        DyLink::ConstraintForceArray& constraintForces = link->constraintForces();
        if(!constraintForces.empty()){
            if(!put){
                os << body->name() << ":\n";
                put = true;
            }
            os << " " << link->name() << ":\n";
            for(size_t i=0; i < constraintForces.size(); ++i){
                const DyLink::ConstraintForce& c = constraintForces[i];
                const Vector3& p = c.point;
                const Vector3& f = c.force;
                os << "  point(" << p.x() << ", " << p.y() << ", " << p.z()
                   << "), force(" << f.x() << ", " << f.y() << ", " << f.z() << ")\n";
            }
        }
    }
}
