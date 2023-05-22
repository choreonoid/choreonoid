#include "BodySuperimposerAddon.h"
#include "BodyItem.h"
#include "OperableSceneBody.h"
#include <cnoid/ItemManager>
#include <cnoid/SceneBody>
#include <cnoid/Referenced>
#include <cnoid/CloneMap>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
//#include <cnoid/PutPropertyFunction>
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
    typedef vector<Isometry3, Eigen::aligned_allocator<Isometry3>> PositionArray;
    PositionArray tmpLinkPositions;
    vector<double> tmpJointDisplacements;
    
};
typedef ref_ptr<BodyInfo> BodyInfoPtr;

}

namespace cnoid {

class BodySuperimposerAddon::Impl
{
public:
    BodySuperimposerAddon* self;
    BodyItem* bodyItem;
    ScopedConnectionSet bodyItemConnections;
    vector<BodyInfoPtr> bodyInfos;
    typedef vector<Isometry3, Eigen::aligned_allocator<Isometry3>> PositionArray;
    bool needToCheckSuperimposedBodies;
    SgGroupPtr topGroup;
    float transparency;
    SgUpdate sgUpdate;
    CloneMap cloneMap;

    Impl(BodySuperimposerAddon* self);
    Impl(BodySuperimposerAddon* self, const Impl& org);
    void setBodyItem(BodyItem* newBodyItem);
    void addSuperimposedBodies(BodyItem* bodyItem, bool isActive);
    void updateSuperimposedBodies();
    bool checkBodySetChange(Item* parentItem, set<BodyItem*>& bodyItemSet);
    void setTransparency(float t);
    void updateSuperimposition();
    bool updateSuperimposition(
        std::function<bool()>& setReferenceConfigurationToOrgBodiesTransiently);
    bool checkPositionIdentity(Body* body1, Body* body2);
};

}


void BodySuperimposerAddon::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAddon<BodySuperimposerAddon>(N_("BodySuperimposer"));
}


BodySuperimposerAddon::BodySuperimposerAddon()
{
    impl = new Impl(this);
}


BodySuperimposerAddon::Impl::Impl(BodySuperimposerAddon* self)
    : self(self)
{
    bodyItem = nullptr;
    needToCheckSuperimposedBodies = false;
    topGroup = new SgGroup;
    transparency = 0.5f;
    SgObject::setNonNodeCloning(cloneMap, false);
}


BodySuperimposerAddon::BodySuperimposerAddon(const BodySuperimposerAddon& org)
    : BodySuperimposerAddon()
{
    impl->transparency = org.impl->transparency;
}


BodySuperimposerAddon::~BodySuperimposerAddon()
{
    delete impl;
}


ItemAddon* BodySuperimposerAddon::doClone(Item* /* newItem */, CloneMap* /* cloneMap */) const
{
    return new BodySuperimposerAddon(*this);
}


bool BodySuperimposerAddon::setOwnerItem(Item* item)
{
    if(auto bodyItem = dynamic_cast<BodyItem*>(item)){
        ItemAddon::setOwnerItem(bodyItem);
        impl->setBodyItem(bodyItem);
        return true;
    }
    return false;
}


void BodySuperimposerAddon::Impl::setBodyItem(BodyItem* bodyItem)
{
    this->bodyItem = bodyItem;

    topGroup->clearChildren();
    bodyInfos.clear();
    bodyItemConnections.disconnect();
    needToCheckSuperimposedBodies = true;
}


void BodySuperimposerAddon::Impl::updateSuperimposedBodies()
{
    if(!needToCheckSuperimposedBodies){
        return;
    }
    needToCheckSuperimposedBodies = false;

    bool bodiesChanged = bodyInfos.empty();

    if(!bodiesChanged){
        set<BodyItem*> bodyItemSet;
        for(size_t i=1; i < bodyInfos.size(); ++i){
            auto& info = bodyInfos[i];
            auto bodyItem = info->bodyItem.lock();
            if(bodyItem && bodyItem->isConnectedToRoot()){
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
    }
    
    if(!bodiesChanged){
        return;
    }

    topGroup->clearChildren();
    bodyInfos.clear();
    bodyItemConnections.disconnect();

    bodyItemConnections.add(
        bodyItem->sigKinematicStateChanged().connect(
            [&](){ self->clearSuperimposition(); }));
    bodyItemConnections.add(
        bodyItem->sigSubTreeChanged().connect(
            [&](){ needToCheckSuperimposedBodies = true; } ));
    
    addSuperimposedBodies(bodyItem, true);
    cloneMap.clear();
}


bool BodySuperimposerAddon::Impl::checkBodySetChange(Item* parentItem, set<BodyItem*>& bodyItemSet)
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


void BodySuperimposerAddon::Impl::addSuperimposedBodies(BodyItem* bodyItem, bool isActive)
{
    bodyItemConnections.add(
        bodyItem->sigUpdated().connect(
            [&](){
                needToCheckSuperimposedBodies = true;
            } ));

    if(isActive){
        BodyInfoPtr info = new BodyInfo;
        info->bodyItem = bodyItem;
        info->superimposedBody = bodyItem->body()->clone(cloneMap);
        info->sceneBody = new SceneBody(info->superimposedBody);
        info->sceneBody->setTransparency(transparency);
        topGroup->addChild(info->sceneBody);
        bodyInfos.push_back(info);
    }

    for(Item* childItem = bodyItem->childItem(); childItem; childItem = childItem->nextItem()){
        if(auto childBodyItem = dynamic_cast<BodyItem*>(childItem)){
            addSuperimposedBodies(childBodyItem, isActive && childBodyItem->isAttachedToParentBody());
        }
    }
}


int BodySuperimposerAddon::numSuperimposedBodies() const
{
    impl->updateSuperimposedBodies();
    return impl->bodyInfos.size();
}


Body* BodySuperimposerAddon::superimposedBody(int index)
{
    impl->updateSuperimposedBodies();
    
    if(index < impl->bodyInfos.size()){
        return impl->bodyInfos[index]->superimposedBody;
    }
    return nullptr;
}


void BodySuperimposerAddon::setTransparency(float transparency)
{
    impl->setTransparency(transparency);
}


void BodySuperimposerAddon::Impl::setTransparency(float t)
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


void BodySuperimposerAddon::updateSuperimposition()
{
    impl->updateSuperimposition();
}


void BodySuperimposerAddon::Impl::updateSuperimposition()
{
    updateSuperimposedBodies();
    
    if(!bodyInfos.empty()){
        bool isDifferent = false;
        for(auto& info : bodyInfos){
            if(auto bodyItem = info->bodyItem.lock()){
                if(!checkPositionIdentity(bodyItem->body(), info->superimposedBody)){
                    isDifferent = true;
                    break;
                }
            }
        }
        if(isDifferent){
            for(auto& info : bodyInfos){
                info->sceneBody->updateLinkPositions(sgUpdate.withAction(SgUpdate::MODIFIED));
            }
        }
        auto sceneBody = bodyItem->sceneBody();
        if(isDifferent){
            if(sceneBody->addChildOnce(topGroup)){
                sceneBody->notifyUpdate(sgUpdate.withAction(SgUpdate::ADDED));
            }
        } else {
            // Superimposition is disabled when the position is same as the original body
            if(sceneBody->removeChild(topGroup)){
                sceneBody->notifyUpdate(sgUpdate.withAction(SgUpdate::REMOVED));
            }
        }
    }
}


bool BodySuperimposerAddon::Impl::checkPositionIdentity(Body* body1, Body* body2)
{
    const int n = body1->numLinks();
    if(n != body2->numLinks()){
        return false;
    }
    for(int i=0; i < n; ++i){
        if(!body1->link(i)->T().isApprox(body2->link(i)->T())){
            return false;
        }
    }
    return true;
}


bool BodySuperimposerAddon::updateSuperimposition
(std::function<bool()> setReferenceConfigurationToOrgBodiesTransiently)
{
    return impl->updateSuperimposition(setReferenceConfigurationToOrgBodiesTransiently);
}


bool BodySuperimposerAddon::Impl::updateSuperimposition
(std::function<bool()>& setReferenceConfigurationToOrgBodiesTransiently)
{
    bool updated = false;

    updateSuperimposedBodies();
    
    // store the original body position
    for(size_t i=0; i < bodyInfos.size(); ++i){
        auto& info = bodyInfos[i];
        auto body = info->bodyItem.lock()->body();
        const int numLinks = body->numLinks();
        auto& linkPositions = info->tmpLinkPositions;
        linkPositions.resize(numLinks);
        for(int j=0; j < numLinks; ++j){
            linkPositions[j] = body->link(j)->T();
        }
        const int numJoints = body->numAllJoints();
        auto& jointDisplacements = info->tmpJointDisplacements;
        jointDisplacements.resize(numJoints);
        for(int j=0; j < numJoints; ++j){
            jointDisplacements[j] = body->joint(j)->q();
        }
    }
    
    if(setReferenceConfigurationToOrgBodiesTransiently()){
        // Copy the body position to the supoerimpose body
        for(size_t i=0; i < bodyInfos.size(); ++i){
            auto& info = bodyInfos[i];
            auto orgBody = info->bodyItem.lock()->body();
            auto superBody = info->superimposedBody;
            auto& linkPositions = info->tmpLinkPositions;
            const int numLinks = orgBody->numLinks();
            for(int j=0; j < numLinks; ++j){
                superBody->link(j)->setPosition(orgBody->link(j)->position());
                // Restore the original link positions
                orgBody->link(j)->setPosition(linkPositions[j]);
            }
            // Restore the original joint displacements
            const int numJoints = orgBody->numAllJoints();
            auto& jointDisplacements = info->tmpJointDisplacements;
            for(int j=0; j < numJoints; ++j){
                orgBody->joint(j)->q() = jointDisplacements[j];
            }
            if(i > 0){
                superBody->syncPositionWithParentBody();
            }
        }
        updateSuperimposition();
        updated = true;
    }

    return updated;
}


void BodySuperimposerAddon::clearSuperimposition()
{
    if(impl->topGroup->hasParents()){
        auto sceneBody = impl->bodyItem->sceneBody();
        sceneBody->removeChild(impl->topGroup);
        sceneBody->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::REMOVED));
    }
}


/*
void BodySuperimposerAddon::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.min(0.0).max(0.9).decimals(1);
    putProperty(_("Transparency"), impl->transparency,
                [&](float value){ impl->setTransparency(value); return true; });
}
*/


bool BodySuperimposerAddon::store(Archive& archive)
{
    archive.write("transparency", impl->transparency);
    return true;
}


bool BodySuperimposerAddon::restore(const Archive& archive)
{
    impl->setTransparency(archive.get("transparency", impl->transparency));
    return true;
}
