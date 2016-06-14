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

using namespace std;
using namespace cnoid;

namespace {

class SpringDamperContactItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    SpringDamperContactItem();
    SpringDamperContactItem(const SpringDamperContactItem& org);
    virtual void onPositionChanged();
    virtual bool initializeSimulation(SimulatorItem* simulatorItem);

protected:
    virtual Item* doDuplicate() const;

private:
    bool calcContactForce(Link* link1, Link* link2, const CollisionArray& collisions);
};


class SpringDamperContactPlugin : public Plugin
{
public:
    SpringDamperContactPlugin() : Plugin("SpringDamperContact") {
        require("Body");
    }

    virtual bool initialize() {
        itemManager()
            .registerClass<SpringDamperContactItem>("SpringDamperContactItem")
            .addCreationPanel<SpringDamperContactItem>();
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SpringDamperContactPlugin);



SpringDamperContactItem::SpringDamperContactItem()
{

}


SpringDamperContactItem::SpringDamperContactItem(const SpringDamperContactItem& org)
    : SubSimulatorItem(org)
{

}


Item* SpringDamperContactItem::doDuplicate() const
{
    return new SpringDamperContactItem(*this);
}


void SpringDamperContactItem::onPositionChanged()
{
    AISTSimulatorItem* simulator = findOwnerItem<AISTSimulatorItem>();
    if(simulator){
        simulator->registerCollisionHandler(
            "SpringDamperContact",
            boost::bind(&SpringDamperContactItem::calcContactForce, this, _1, _2, _3));
    }
}


bool SpringDamperContactItem::initializeSimulation(SimulatorItem* simulatorItem)
{

}


bool SpringDamperContactItem::calcContactForce(Link* link1, Link* link2, const CollisionArray& collisions)
{
    ostream& os = mvout();
    os << link1->name() << " <-> " << link2->name() << "\n";
    for(size_t i=0; i < collisions.size(); ++i){
        const Collision& c = collisions[i];
        os << " point: " << c.point << "\n";
        os << " normal: " << c.normal << "\n";
        os << " depth: " << c.depth << "\n";
    }
    os.flush();
    return false;
}
