/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/SubSimulatorItem>
#include <cnoid/AISTSimulatorItem>
#include <cnoid/ContactMaterial>

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
    bool calcContactForce(Link* link1, Link* link2, const CollisionArray& collisions, ContactMaterial* cm);

    weak_ref_ptr<AISTSimulatorItem> weakCurrentSimulator;
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
    AISTSimulatorItem* currentSimulator = weakCurrentSimulator.lock();
    if(simulator != currentSimulator){
        if(currentSimulator){
            currentSimulator->unregisterCollisionHandler("SpringDamperContact");
            weakCurrentSimulator.reset();
        }
        if(simulator){
            simulator->registerCollisionHandler(
                "SpringDamperContact",
                [&](Link* link1, Link* link2, const CollisionArray& collisions, ContactMaterial* cm){
                    return calcContactForce(link1, link2, collisions, cm); });
            
            weakCurrentSimulator = simulator;
        }
    }
}


bool SpringDamperContactItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    return true;
}


bool SpringDamperContactItem::calcContactForce
(Link* link1, Link* link2, const CollisionArray& collisions, ContactMaterial* cm)
{
    const bool doApplyToLink1 = !link1->isRoot() || !link1->isFixedJoint();
    const bool doApplyToLink2 = !link2->isRoot() || !link2->isFixedJoint();
    const double mu = cm->dynamicFriction();

    for(size_t i=0; i < collisions.size(); ++i){
        const Collision& c = collisions[i];

        const Vector3 v1 = link1->v() + link1->w().cross(c.point - link1->p());
        const Vector3 v2 = link2->v() + link2->w().cross(c.point - link2->p());
        const Vector3 v = v2 - v1;
        const double vn = v.dot(c.normal);
        const double kn = (c.depth * 100000.0 - vn * 1000.0);
        const Vector3 fn = kn * c.normal;
        const Vector3 vt = (v - vn * c.normal);
        const double vt_thresh = 0.01;
        Vector3 tangent;
        if(vt.squaredNorm() > vt_thresh * vt_thresh){
            tangent = vt.normalized();
        } else {
            tangent = vt;
        }
        const Vector3 ft = -mu * kn * tangent;
        const Vector3 f = fn + ft;
        
        if(doApplyToLink1){
            link1->f_ext() += -f;
            link1->tau_ext() += c.point.cross(-f);
        }
        if(doApplyToLink2){
            link2->f_ext() += f;
            link2->tau_ext() += c.point.cross(f);
        }
    }
    return true;
}
