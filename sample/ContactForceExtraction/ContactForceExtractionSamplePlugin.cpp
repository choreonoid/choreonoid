/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/SimulatorItem>
#include <cnoid/SubSimulatorItem>
#include <cnoid/Body>
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
    void extractBodyContactPoints(Body* body, ostream& os);
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
    for(auto& simBody : simulator->simulationBodies()){
        for(auto& link : simBody->body()->links()){
            link->mergeSensingMode(Link::LinkContactState);
        }
    }

    simulator->addPostDynamicsFunction(
        [this, simulator](){ extractContactPoints(simulator); });

    return true;
}


void ContactForceExtractorItem::extractContactPoints(SimulatorItem* simulator)
{
    ostringstream oss;
    for(auto& simBody : simulator->simulationBodies()){
        extractBodyContactPoints(simBody->body(), oss);
    }
    string log = oss.str();
    if(!log.empty()){
        mvout() << log << endl;
    }
}


void ContactForceExtractorItem::extractBodyContactPoints(Body* body, ostream& os)
{
    bool put = false;
    for(auto& link : body->links()){
        auto& contacts = link->contactPoints();
        if(!contacts.empty()){
            if(!put){
                os << body->name() << ":\n";
                put = true;
            }
            os << " " << link->name() << ":\n";
            for(auto& contact : contacts){
                const Vector3& p = contact.position();
                const Vector3& f = contact.force();
                os << "  point(" << p.x() << ", " << p.y() << ", " << p.z()
                   << "), force(" << f.x() << ", " << f.y() << ", " << f.z() << ")\n";
            }
        }
    }
}
