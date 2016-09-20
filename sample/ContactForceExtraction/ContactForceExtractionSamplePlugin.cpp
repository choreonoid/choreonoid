/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/SubSimulatorItem>
#include <cnoid/AISTSimulatorItem>
#include <cnoid/DyBody>
#include <sstream>

using namespace std;
using namespace cnoid;

namespace {

class ContactForceExtractorItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ContactForceExtractorItem();
    ContactForceExtractorItem(const ContactForceExtractorItem& org);

    virtual bool initializeSimulation(SimulatorItem* simulatorItem);

protected:
    virtual Item* doDuplicate() const;

private:
    void extractContactPoints(SimulatorItem* simulator);
    void extractBodyContactPoints(DyBody* body, ostream& os);
};


class ContactForceExtractionSamplePlugin : public Plugin
{
public:
    ContactForceExtractionSamplePlugin() : Plugin("ContactForceExtractionSample") {
        require("Body");
    }

    virtual bool initialize() {
        itemManager()
            .registerClass<ContactForceExtractorItem>("ContactForceExtractorItem")
            .addCreationPanel<ContactForceExtractorItem>();
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ContactForceExtractionSamplePlugin);



ContactForceExtractorItem::ContactForceExtractorItem()
{

}


ContactForceExtractorItem::ContactForceExtractorItem(const ContactForceExtractorItem& org)
    : SubSimulatorItem(org)
{

}


Item* ContactForceExtractorItem::doDuplicate() const
{
    return new ContactForceExtractorItem(*this);
}


bool ContactForceExtractorItem::initializeSimulation(SimulatorItem* simulator)
{
    AISTSimulatorItem* aistSimulator = dynamic_cast<AISTSimulatorItem*>(simulator);
    if(!aistSimulator){
        return false;
    }

    aistSimulator->setConstraintForceOutputEnabled(true);

    simulator->addPostDynamicsFunction(
        std::bind(&ContactForceExtractorItem::extractContactPoints, this, simulator));

    return true;
}


void ContactForceExtractorItem::extractContactPoints(SimulatorItem* simulator)
{
    ostringstream oss;

    const vector<SimulationBody*>& simBodies = simulator->simulationBodies();
    for(size_t i=0; i < simBodies.size(); ++i){
        DyBody* body = dynamic_cast<DyBody*>(simBodies[i]->body());
        if(body){
            extractBodyContactPoints(body, oss);
        }
    }
    string log = oss.str();
    if(!log.empty()){
        mvout() << log << endl;
    }
}


void ContactForceExtractorItem::extractBodyContactPoints(DyBody* body, ostream& os)
{
    bool put = false;
    for(int i=0; i < body->numLinks(); ++i){
        DyLink* link = body->link(i);
        DyLink::ConstraintForceArray& forces = link->constraintForces();
        if(!forces.empty()){
            if(!put){
                os << body->name() << ":\n";
                put = true;
            }
            os << " " << link->name() << ":\n";
            for(size_t i=0; i < forces.size(); ++i){
                const DyLink::ConstraintForce& force = forces[i];
                const Vector3& p = force.point;
                const Vector3& f = force.force;
                os << "  point(" << p.x() << ", " << p.y() << ", " << p.z()
                   << "), force(" << f.x() << ", " << f.y() << ", " << f.z() << ")\n";
            }
        }
    }
}
