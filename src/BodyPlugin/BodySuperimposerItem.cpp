#include "BodySuperimposerItem.h"
#include "BodyItem.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/SceneBody>
#include <cnoid/CloneMap>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodySuperimposerItem::Impl
{
public:
    BodySuperimposerItem* self;
    BodyItem* bodyItem;
    ScopedConnectionSet bodyItemConnections;
    vector<BodyPtr> superimposedBodies;
    SgSwitchableGroupPtr topSwitch;
    vector<SceneBodyPtr> sceneBodies;
    float transparency;
    SgUpdate sgUpdate;
    CloneMap cloneMap;

    Impl(BodySuperimposerItem* self);
    Impl(BodySuperimposerItem* self, const Impl& org);
    void setBodyItem(BodyItem* newBodyItem);
    void addSuperimposedBodies(BodyItem* bodyItem);
    void setTransparency(float t);
    void updateSuperimposition();
};

}


void BodySuperimposerItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BodySuperimposerItem>(N_("BodySuperimposerItem"));
}


BodySuperimposerItem::BodySuperimposerItem()
{
    impl = new Impl(this);
}


BodySuperimposerItem::Impl::Impl(BodySuperimposerItem* self)
    : self(self)
{
    bodyItem = nullptr;
    topSwitch = new SgSwitchableGroup;
    topSwitch->setTurnedOn(false);
    transparency = 0.5;
    SgObject::setNonNodeCloning(cloneMap, false);
}


BodySuperimposerItem::BodySuperimposerItem(const BodySuperimposerItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
    setName(org.name());
}


BodySuperimposerItem::Impl::Impl(BodySuperimposerItem* self, const Impl& org)
    : Impl(self)
{
    transparency = org.transparency;
}

    

BodySuperimposerItem::~BodySuperimposerItem()
{
    delete impl;
}


Item* BodySuperimposerItem::doDuplicate() const
{
    return new BodySuperimposerItem(*this);
}


void BodySuperimposerItem::setName(const std::string& name)
{
    Item::setName(name);
    impl->topSwitch->setName(name);
}


void BodySuperimposerItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>());
}


void BodySuperimposerItem::Impl::setBodyItem(BodyItem* newBodyItem)
{
    if(newBodyItem != bodyItem){
        bodyItem = newBodyItem;
        superimposedBodies.clear();
        topSwitch->clearChildren();
        sceneBodies.clear();
        bodyItemConnections.disconnect();

        if(bodyItem){
            addSuperimposedBodies(bodyItem);
            cloneMap.clear();

            bodyItemConnections.add(
                bodyItem->sigKinematicStateChanged().connect(
                    [&](){ self->clearSuperimposition(); }));
            bodyItemConnections.add(
                bodyItem->sigCheckToggled().connect(
                    [&](bool on){ self->setChecked(on); }));

            self->setChecked(bodyItem->isChecked());
        }
    }
}


void BodySuperimposerItem::Impl::addSuperimposedBodies(BodyItem* bodyItem)
{
    auto superimposedBody = bodyItem->body()->clone(cloneMap);
    superimposedBodies.push_back(superimposedBody);
    auto sceneBody = new SceneBody(superimposedBody);
    sceneBody->makeTransparent(transparency, cloneMap);
    topSwitch->addChild(sceneBody);
    sceneBodies.push_back(sceneBody);

    for(Item* childItem = bodyItem->childItem(); childItem; childItem = childItem->nextItem()){
        if(auto childBodyItem = dynamic_cast<BodyItem*>(childItem)){
            if(childBodyItem->isAttachedToParentBody()){
                addSuperimposedBodies(childBodyItem);
            }
        }
    }
}
    

int BodySuperimposerItem::numSuperimposedBodies() const
{
    return impl->superimposedBodies.size();
}


Body* BodySuperimposerItem::superimposedBody(int index)
{
    if(index < impl->superimposedBodies.size()){
        return impl->superimposedBodies[index];
    }
    return nullptr;
}


SgNode* BodySuperimposerItem::getScene()
{
    return impl->topSwitch;
}
    

void BodySuperimposerItem::setTransparency(float transparency)
{
    impl->setTransparency(transparency);
}


void BodySuperimposerItem::Impl::setTransparency(float t)
{
    if(t != transparency){
        if(!sceneBodies.empty()){
            for(auto& sceneBody : sceneBodies){
                sceneBody->makeTransparent(t, cloneMap);
            }
            cloneMap.clear();
        }
        transparency = t;
    }
}


void BodySuperimposerItem::updateSuperimposition()
{
    impl->updateSuperimposition();
}


void BodySuperimposerItem::Impl::updateSuperimposition()
{
    if(!sceneBodies.empty()){
        for(auto& sceneBody : sceneBodies){
            sceneBody->updateLinkPositions(sgUpdate);
        }
        topSwitch->setTurnedOn(true);
        topSwitch->notifyUpdate(sgUpdate);
    }
}


void BodySuperimposerItem::clearSuperimposition()
{
    impl->topSwitch->setTurnedOn(false, true);
}


void BodySuperimposerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.min(0.0).max(1.0);
    putProperty(_("Transparency"), static_cast<double>(impl->transparency),
                [&](double value){ impl->setTransparency(value); return true; });
}


bool BodySuperimposerItem::store(Archive& archive)
{
    archive.write("transparency", impl->transparency);
    return true;
}


bool BodySuperimposerItem::restore(const Archive& archive)
{
    impl->setTransparency(archive.get("transparency", impl->transparency));
    return true;
}
