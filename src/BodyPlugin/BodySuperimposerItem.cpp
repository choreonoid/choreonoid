#include "BodySuperimposerItem.h"
#include "BodyItem.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/SceneBody>
#include <cnoid/CloneMap>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

struct BodyInfo : public Referenced
{
    weak_ref_ptr<BodyItem> bodyItem;
    BodyPtr superimposedBody;
    SceneBodyPtr sceneBody;
};
typedef ref_ptr<BodyInfo> BodyInfoPtr;

}

namespace cnoid {

class BodySuperimposerItem::Impl
{
public:
    BodySuperimposerItem* self;
    BodyItem* bodyItem;
    ScopedConnectionSet bodyItemConnections;
    vector<BodyInfoPtr> bodyInfos;
    bool needToUpdateSuperimposedBodies;
    SgSwitchableGroupPtr topSwitch;
    float transparency;
    SgUpdate sgUpdate;
    CloneMap cloneMap;

    Impl(BodySuperimposerItem* self);
    Impl(BodySuperimposerItem* self, const Impl& org);
    void setBodyItem(BodyItem* newBodyItem);
    void addSuperimposedBodies(BodyItem* bodyItem);
    void updateSuperimposedBodies();
    bool checkBodySetChange(Item* parentItem, set<BodyItem*>& bodyItemSet);
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
    needToUpdateSuperimposedBodies = false;
    topSwitch = new SgSwitchableGroup;
    topSwitch->setTurnedOn(false);
    transparency = 0.5f;
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
        topSwitch->clearChildren();
        bodyInfos.clear();
        bodyItemConnections.disconnect();

        if(bodyItem){
            needToUpdateSuperimposedBodies = true;

            bodyItemConnections.add(
                bodyItem->sigKinematicStateChanged().connect(
                    [&](){ self->clearSuperimposition(); }));
            bodyItemConnections.add(
                bodyItem->sigCheckToggled().connect(
                    [&](bool on){ self->setChecked(on); }));
            bodyItemConnections.add(
                bodyItem->sigSubTreeChanged().connect(
                    [&](){ needToUpdateSuperimposedBodies = true; } ));

            self->setChecked(bodyItem->isChecked());
        }
    }
}


void BodySuperimposerItem::Impl::updateSuperimposedBodies()
{
    if(!needToUpdateSuperimposedBodies){
        return;
    }
    needToUpdateSuperimposedBodies = false;

    bool bodiesChanged = false;
    set<BodyItem*> bodyItemSet;
    for(size_t i=1; i < bodyInfos.size(); ++i){
        auto& info = bodyInfos[i];
        if(BodyItem* bodyItem = info->bodyItem.lock()){
            bodyItemSet.insert(bodyItem);
        } else {
            bodiesChanged = true;
            break;
        }
    }
    if(!bodiesChanged){
        if(checkBodySetChange(bodyItem, bodyItemSet) || !bodyItemSet.empty()){
            bodiesChanged = true;
        }
    }
    if(!bodiesChanged){
        return;
    }
    
    bodyInfos.clear();
    topSwitch->clearChildren();
    addSuperimposedBodies(bodyItem);
    cloneMap.clear();
}


bool BodySuperimposerItem::Impl::checkBodySetChange(Item* parentItem, set<BodyItem*>& bodyItemSet)
{
    for(Item* childItem = parentItem->childItem(); childItem; childItem = childItem->nextItem()){
        if(auto childBodyItem = dynamic_cast<BodyItem*>(childItem)){
            if(childBodyItem->isAttachedToParentBody()){
                auto p = bodyItemSet.find(childBodyItem);
                if(p == bodyItemSet.end()){
                    return true;
                }
                bodyItemSet.erase(p);
            }
        }
        if(checkBodySetChange(childItem, bodyItemSet)){
            return true;
        }
    }
    return false;
}


void BodySuperimposerItem::Impl::addSuperimposedBodies(BodyItem* bodyItem)
{
    BodyInfoPtr info = new BodyInfo;
    info->bodyItem = bodyItem;
    info->superimposedBody = bodyItem->body()->clone(cloneMap);
    info->sceneBody = new SceneBody(info->superimposedBody);
    info->sceneBody->setTransparency(transparency);
    topSwitch->addChild(info->sceneBody);
    bodyInfos.push_back(info);

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
    impl->updateSuperimposedBodies();
    return impl->bodyInfos.size();
}


Body* BodySuperimposerItem::superimposedBody(int index)
{
    impl->updateSuperimposedBodies();
    
    if(index < impl->bodyInfos.size()){
        return impl->bodyInfos[index]->superimposedBody;
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
        if(!bodyInfos.empty()){
            for(auto& info : bodyInfos){
                info->sceneBody->setTransparency(t);
            }
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
    updateSuperimposedBodies();
    
    if(!bodyInfos.empty()){
        for(auto& info : bodyInfos){
            info->sceneBody->updateLinkPositions(sgUpdate);
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
    putProperty.min(0.0).max(0.9).decimals(1);
    putProperty(_("Transparency"), impl->transparency,
                [&](float value){ impl->setTransparency(value); return true; });
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
