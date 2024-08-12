#include "LinkOverwriteItem.h"
#include "BodyOverwriteAddon.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/StdBodyLoader>
#include <cnoid/StdBodyWriter>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneNodeExtractor>
#include <cnoid/StdSceneReader>
#include <cnoid/StdSceneWriter>
#include <cnoid/PositionDragger>
#include <cnoid/SceneUtil>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/Archive>
#include <cnoid/CloneMap>
#include <cnoid/EigenArchive>
#include <cnoid/MessageView>
#include <cnoid/Format>
#include <unordered_map>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

weak_ptr<StdSceneReader> sharedSceneReader;
weak_ptr<StdSceneWriter> sharedSceneWriter;

class OffsetLocation : public LocationProxy
{
public:
    LinkOverwriteItem::Impl* impl;
    mutable LocationProxyPtr parentLinkLocation;
    mutable LocationProxyPtr linkLocation;
    Signal<void()> sigLocationChanged_;

    OffsetLocation(LinkOverwriteItem::Impl* impl);
    bool isLinkOffsetLocation() const;
    bool isShapeOffsetLocation() const;
    virtual std::string getName() const override;
    virtual Isometry3 getLocation() const override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual Item* getCorrespondingItem() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

typedef ref_ptr<OffsetLocation> OffsetLocationPtr;    

}

namespace cnoid {

class LinkOverwriteItem::Impl
{
public:
    LinkOverwriteItem* self;
    int overwriteElementSet;
    LinkPtr targetLink;
    LinkPtr referenceLink;
    LinkPtr additionalLink;
    string additionalLinkParentName;
    LinkPtr originalLinkClone;
    bool isRootLink;

    Isometry3 shapeOffset;
    SgPosTransformPtr shapeOffsetTransform;
    SgPosTransformPtr collisionShapeOffsetTransform;
    Vector3f shapeColor;

    struct MaterialInfo
    {
        SgMaterialPtr orgMaterialClone;
        bool isUpdated;
    };
    unordered_map<SgMaterialPtr, MaterialInfo> materialMap;
    unordered_set<SgShapePtr> noMaterialShapes;
    SgMaterialPtr materialForNoMaterialShapes;
    
    SgNodePtr visualShape;
    SgNodePtr collisionShape;
    SgUpdate sgUpdate;

    PositionDraggerPtr originMarker;
    OffsetLocationPtr offsetLocation;
    int locationTargetType;
    ScopedConnection bodyItemConnection;

    shared_ptr<StdSceneReader> sceneReader;
    shared_ptr<StdSceneWriter> sceneWriter;

    Impl(LinkOverwriteItem* self);
    Impl(LinkOverwriteItem* self, const Impl& org, CloneMap* cloneMap);
    bool updateOverwriting(BodyItem* bodyItem);
    void overwriteExistingLink(Link* existingLink);
    void copyOverwriteLinkElements(Link* srcLink, Link* destLink);
    void overwriteShapeElements(Link* link);
    void cancelOverwritingShapeElements(Link* link);
    void overwriteShapeOffset(Link* link, bool doNotify);
    void cancelOverwritingShapeOffset(Link* link, bool doNotify);
    void overwriteShapeColor(Link* link, bool doNotify);
    void cancelOverwritingShapeColor(Link* link, bool doNotify);
    void restoreOriginalShapeMaterials(Link* link, bool doNotify);
    void restoreOverwritingShapeMaterials(Link* link, bool doNotify);
    void notifyMaterialUpdates(int noMaterialShapeAction);
    bool addNewLink(Body* body);
    void releaseOverwriteTarget();
    void cancelOverwriting();
    bool restoreShapeWrittenInOldFormat(const Archive& archive, ValueNode* shapeArchive);
    void setReferenceLinkToRestoreShapeWritteinInOldFormat(Link* orgLink, SgNode* newShape);
    SgPosTransform* extractOrInsertOffsetTransform(vector<SgNodePath>& nodePaths);
    void updateLinkOriginMarker();
    bool store(Archive& archive);
    shared_ptr<StdSceneReader> ensureSceneReader(const Archive& archive);
    bool restore(const Archive& archive);
};

}


void LinkOverwriteItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<LinkOverwriteItem, BodyElementOverwriteItem>(N_("LinkOverwriteItem"));
    im.addAlias<LinkOverwriteItem>("LinkShapeOverwriteItem", "Body");
    im.addAlias<LinkOverwriteItem>("LinkShapeOverwriteItem", "BodyEdit");
}


LinkOverwriteItem* LinkOverwriteItem::findLinkOverwriteItem(BodyItem* bodyItem, Link* link)
{
    return bodyItem->getAddon<BodyOverwriteAddon>()->findLinkOverwriteItem(link);
}


LinkOverwriteItem::LinkOverwriteItem()
{
    setAttribute(Item::Attached);
    impl = new Impl(this);
}


LinkOverwriteItem::Impl::Impl(LinkOverwriteItem* self)
    : self(self)
{
    overwriteElementSet = NoElement;
    isRootLink = false;
    shapeOffset.setIdentity();
    shapeColor.setOnes();
    locationTargetType = LinkOffsetLocation;
}


LinkOverwriteItem::LinkOverwriteItem(const LinkOverwriteItem& org, CloneMap* cloneMap)
    : BodyElementOverwriteItem(org, cloneMap)
{
    impl = new Impl(this, *org.impl, cloneMap);
}


LinkOverwriteItem::Impl::Impl(LinkOverwriteItem* self, const Impl& org, CloneMap* cloneMap)
    : self(self),
      overwriteElementSet(org.overwriteElementSet),
      additionalLinkParentName(org.additionalLinkParentName)
{
    if(org.targetLink && cloneMap && self->bodyItem()){
        targetLink = cloneMap->findClone(org.targetLink);
    }
    if(org.referenceLink){
        referenceLink = CloneMap::getClone(org.referenceLink, cloneMap);
    }
    if(org.additionalLink){
        additionalLink = CloneMap::getClone(org.additionalLink, cloneMap);
    }
    isRootLink = org.isRootLink;
    locationTargetType = org.locationTargetType;

    shapeOffset = org.shapeOffset;
    shapeColor = org.shapeColor;

    if(org.visualShape){
        visualShape = CloneMap::getClone(org.visualShape, cloneMap);
    }
    if(org.collisionShape){
        collisionShape = CloneMap::getClone(org.collisionShape, cloneMap);
    }
}


LinkOverwriteItem::~LinkOverwriteItem()
{
    delete impl;
}


Item* LinkOverwriteItem::doCloneItem(CloneMap* cloneMap) const
{
    return new LinkOverwriteItem(*this, cloneMap);
}


bool LinkOverwriteItem::onNewOverwritePositionCheck(bool /* isManualOperation */)
{
    bool acceptable = false;
    auto newBodyItem_ = newBodyItem();
    if(impl->updateOverwriting(newBodyItem_)){
        setBodyItem(newBodyItem_);
        acceptable = true;
    }

    return acceptable;
}


void LinkOverwriteItem::onDisconnectedFromBodyItem()
{
    impl->cancelOverwriting();
}


bool LinkOverwriteItem::setName(const std::string& name)
{
    auto bodyItem_ = bodyItem();
    auto sLink = sourceLink();
    auto tLink = targetLink();
    if(bodyItem_ && sLink){
        auto body = bodyItem_->body();
        auto existingLink = body->link(name);
        if(existingLink){
            if(existingLink == tLink){
                return true; // Same as the current name
            }
            return false; // Same as the another link name
        }
        sLink->setName(name);
        if(tLink){
            tLink->setName(name);
            body->updateLinkTree();
            bodyItem_->notifyModelUpdate(BodyItem::LinkSetUpdate | BodyItem::LinkSpecUpdate);
        }
    }
    return Item::setName(name);
}


void LinkOverwriteItem::setOverwriteElementSet(int elementSet)
{
    impl->overwriteElementSet = elementSet;
}


void LinkOverwriteItem::addOverwriteElement(int element)
{
    impl->overwriteElementSet |= element;
}


void LinkOverwriteItem::addOverwriteElementSet(int elementSet)
{
    impl->overwriteElementSet |= elementSet;
}    


int LinkOverwriteItem::overwriteElementSet() const
{
    return impl->overwriteElementSet;
}


bool LinkOverwriteItem::hasOverwriteElement(int element) const
{
    return impl->overwriteElementSet & element;
}


bool LinkOverwriteItem::setReferenceLink(Link* referenceLink, bool isRootLink)
{
    if(isOverwriting()){
        return false;
    }
    setName(referenceLink->name());
    impl->referenceLink = referenceLink;
    impl->additionalLink.reset();
    impl->additionalLinkParentName.clear();
    impl->isRootLink = isRootLink;
    return true;
}


Link* LinkOverwriteItem::referenceLink()
{
    return impl->referenceLink;
}


bool LinkOverwriteItem::isRootLink() const
{
    return impl->isRootLink;
}


Link* LinkOverwriteItem::originalLinkClone()
{
    return impl->originalLinkClone;
}


bool LinkOverwriteItem::setAdditionalLink(Link* additionalLink, const std::string& parentLinkName)
{
    if(isOverwriting()){
        return false;
    }
    setName(additionalLink->name());
    impl->additionalLink = additionalLink;
    impl->additionalLinkParentName = parentLinkName;
    impl->referenceLink.reset();
    impl->isRootLink = false;
    return true;
}


Link* LinkOverwriteItem::additionalLink()
{
    return impl->additionalLink;
}


void LinkOverwriteItem::setShapeOffset(const Isometry3& T, bool doOverwrite)
{
    impl->shapeOffset = T;
    impl->overwriteElementSet |= ShapeOffset;
    if(doOverwrite && impl->targetLink){
        impl->overwriteShapeOffset(impl->targetLink, true);
        if(auto bodyItem_ = bodyItem()){
            bodyItem_->notifyModelUpdate(BodyItem::ShapeUpdate);
        }
    }
}


const Isometry3& LinkOverwriteItem::shapeOffset() const
{
    return impl->shapeOffset;
}


void LinkOverwriteItem::setShapeColor(const Vector3f& color, bool doOverwrite)
{
    impl->shapeColor = color;
    impl->overwriteElementSet |= ShapeColor;
    if(doOverwrite && impl->targetLink){
        impl->overwriteShapeColor(impl->targetLink, true);
        if(auto bodyItem_ = bodyItem()){
            bodyItem_->notifyModelUpdate(BodyItem::ShapeUpdate);
        }
    }
}


void LinkOverwriteItem::resetShapeColor(bool doNotify)
{
    impl->cancelOverwritingShapeColor(impl->targetLink, doNotify);
    if(doNotify){
        if(auto bodyItem_ = bodyItem()){
            bodyItem_->notifyModelUpdate(BodyItem::ShapeUpdate);
        }
    }        
}


const Vector3f& LinkOverwriteItem::shapeColor() const
{
    return impl->shapeColor;
}


void LinkOverwriteItem::setShape(SgNode* shape)
{
    setVisualShape(shape);
    setCollisionShape(shape);
}


void LinkOverwriteItem::setVisualShape(SgNode* shape)
{
    impl->overwriteElementSet |= Shape;
    impl->visualShape = shape;
}


void LinkOverwriteItem::setCollisionShape(SgNode* shape)
{
    impl->overwriteElementSet |= Shape;
    impl->collisionShape = shape;
}


SgNode* LinkOverwriteItem::visualShape()
{
    return impl->visualShape;
}


SgNode* LinkOverwriteItem::collisionShape()
{
    return impl->collisionShape;
}


std::string LinkOverwriteItem::findOriginalShapeFile() const
{
    string filename;

    if(impl->targetLink){
        auto pred = [](SgObject* object){ return object->hasUri(); };
        SgObject* uriObject = impl->targetLink->visualShape()->findObject(pred);
        if(!uriObject){
            uriObject = impl->targetLink->collisionShape()->findObject(pred);
        }
        if(uriObject){
            filename = uriObject->localFileAbsolutePath();
            if(filename.empty()){
                filename = uriObject->localFilePath();
            }
        }
    }
        
    return filename;
}


Link* LinkOverwriteItem::sourceLink()
{
    return impl->referenceLink ? impl->referenceLink : impl->additionalLink;
}


Link* LinkOverwriteItem::targetLink()
{
    return impl->targetLink;
}


bool LinkOverwriteItem::isOverwriting() const
{
    return impl->targetLink != nullptr;
}


bool LinkOverwriteItem::isOverwritingExistingLink() const
{
    return impl->targetLink && impl->originalLinkClone;
}


bool LinkOverwriteItem::isAddingLink() const
{
    return impl->targetLink && !impl->originalLinkClone;
}


bool LinkOverwriteItem::updateOverwriting()
{
    return impl->updateOverwriting(bodyItem());
}


bool LinkOverwriteItem::Impl::updateOverwriting(BodyItem* bodyItem)
{
    bodyItemConnection.disconnect();
    
    if(!bodyItem){
        return false;
    }

    bool updated = false;
    auto body = bodyItem->body();
    Link* newTargetLink = nullptr;

    if(targetLink){
        if(referenceLink){
            overwriteExistingLink(targetLink);
            updated = true;
        } else if(additionalLink){
            overwriteShapeElements(targetLink);
            updated = true;
        }
    } else {
        if(referenceLink){
            if(isRootLink){
                newTargetLink = body->rootLink();
            } else {
                newTargetLink = body->link(referenceLink->name());
                if(newTargetLink->isRoot()){
                    isRootLink = true;
                }
            }
        } else if(additionalLink){
            newTargetLink = additionalLink;
        }
        if(newTargetLink){
            auto addon = bodyItem->getAddon<BodyOverwriteAddon>();
            if(addon->registerLinkOverwriteItem(newTargetLink, self)){
                if(referenceLink){
                    overwriteExistingLink(newTargetLink);
                    updated = true;
                } else if(additionalLink){
                    updated = addNewLink(body);
                    overwriteShapeElements(newTargetLink);
                }
                if(updated){
                    targetLink = newTargetLink;
                } else {
                    addon->unregisterLinkOverwriteItem(self);
                }
            }
        }
    }

    if(updated){
        bool isKinematicStateChangeNotificationRequested = false;
        
        if((additionalLink && newTargetLink) ||
           (overwriteElementSet & (JointType | JointId | JointName))){
            body->updateLinkTree();
            if(bodyItem->isBeingRestored()){
                bodyItem->requestNonRootLinkStatesRestorationOnSubTreeRestored();
                isKinematicStateChangeNotificationRequested = true;
            }
        }

        bodyItem->notifyModelUpdate(
            BodyItem::LinkSetUpdate | BodyItem::LinkSpecUpdate |
            BodyItem::DeviceSetUpdate | BodyItem::ShapeUpdate);

        if(!isKinematicStateChangeNotificationRequested){
            if(overwriteElementSet & (OffsetPosition | JointType | JointAxis)){
                bodyItem->notifyKinematicStateChange(true);
            }
        }

        // todo: Execute the following code only when the target body item is changed
        bodyItemConnection =
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ updateLinkOriginMarker(); });
    
        if(originMarker){
            updateLinkOriginMarker();
        }

        if(offsetLocation){
            if(((overwriteElementSet & OffsetPosition) && (locationTargetType == LinkOffsetLocation)) ||
               ((overwriteElementSet & ShapeOffset) && (locationTargetType == ShapeOffsetLocation))){
                offsetLocation->sigLocationChanged_();
            }
        }

        self->notifyUpdate();
    }
    
    return updated;
}


void LinkOverwriteItem::Impl::overwriteExistingLink(Link* existingLink)
{
    if(!originalLinkClone){
        originalLinkClone = existingLink->clone();
    }
    copyOverwriteLinkElements(referenceLink, existingLink);
    overwriteShapeElements(existingLink);
}


void LinkOverwriteItem::Impl::copyOverwriteLinkElements(Link* srcLink, Link* destLink)
{
    if(isRootLink){
        destLink->setName(srcLink->name());
    }
    if(overwriteElementSet & OffsetPosition){
        destLink->setOffsetPosition(srcLink->offsetPosition());
    }
    if(overwriteElementSet & JointType){
        destLink->setJointType(srcLink->jointType());
    }
    if(overwriteElementSet & JointAxis){
        destLink->setJointAxis(srcLink->jointAxis());
    }
    if(overwriteElementSet & JointId){
        destLink->setJointId(srcLink->jointId());
    }
    if(overwriteElementSet & JointName){
        destLink->setJointName(srcLink->jointName());
    }
    if(overwriteElementSet & JointRange){
        destLink->setJointRange(srcLink->q_lower(), srcLink->q_upper());
    }
    if(overwriteElementSet & JointVelocityRange){
        destLink->setJointVelocityRange(srcLink->dq_lower(), srcLink->dq_upper());
    }
    if(overwriteElementSet & JointEffortRange){
        destLink->setJointEffortRange(srcLink->u_lower(), srcLink->u_upper());
    }
    if(overwriteElementSet & Mass){
        destLink->setMass(srcLink->mass());
    }
    if(overwriteElementSet & Inertia){
        destLink->setInertia(srcLink->I());
    }
    if(overwriteElementSet & CenterOfMass){
        destLink->setCenterOfMass(srcLink->centerOfMass());
    }
    if(overwriteElementSet & Material){
        destLink->setMaterial(srcLink->materialId());
    }
}


void LinkOverwriteItem::Impl::overwriteShapeElements(Link* link)
{
    if(overwriteElementSet & Shape){
        link->clearShapeNodes(sgUpdate);
        link->addVisualShapeNode(visualShape, sgUpdate);
        link->addCollisionShapeNode(collisionShape, sgUpdate);
        shapeOffsetTransform.reset();
        collisionShapeOffsetTransform.reset();
    }
    overwriteShapeOffset(link, true);
    overwriteShapeColor(link, true);
}


void LinkOverwriteItem::Impl::cancelOverwritingShapeElements(Link* link)
{
    cancelOverwritingShapeOffset(link, true);
    cancelOverwritingShapeColor(link, true);

    if(overwriteElementSet & Shape){
        if(originalLinkClone){
            link->clearShapeNodes(sgUpdate);
            for(auto& node : *originalLinkClone->visualShape()){
                link->addVisualShapeNode(node, sgUpdate);
            }
            for(auto& node : *originalLinkClone->collisionShape()){
                link->addCollisionShapeNode(node, sgUpdate);
            }
        }
    }
}


void LinkOverwriteItem::Impl::overwriteShapeOffset(Link* link, bool doNotify)
{
    if(overwriteElementSet & ShapeOffset){
        if(shapeOffsetTransform || collisionShapeOffsetTransform){
            if(shapeOffsetTransform){
                shapeOffsetTransform->setPosition(shapeOffset);
                if(doNotify){
                    shapeOffsetTransform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
                }
            }
            if(collisionShapeOffsetTransform && collisionShapeOffsetTransform != shapeOffsetTransform){
                collisionShapeOffsetTransform->setPosition(shapeOffset);
                if(doNotify){
                    collisionShapeOffsetTransform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
                }
            }
        } else {
            bool hasDedicatedCollisionShape = link->hasDedicatedCollisionShape();
            auto visualShape = link->visualShape();
            if(!visualShape->empty()){
                shapeOffsetTransform = new SgPosTransform(shapeOffset);
                visualShape->moveChildrenTo(shapeOffsetTransform);
                visualShape->addChild(shapeOffsetTransform);
                if(doNotify){
                    visualShape->notifyUpdate(sgUpdate.withAction(SgUpdate::Added | SgUpdate::Removed));
                }
            }
            auto collisionShape = link->collisionShape();
            if(!hasDedicatedCollisionShape){
                if(shapeOffsetTransform){
                    collisionShape->clearChildren();
                    collisionShape->addChild(shapeOffsetTransform);
                    collisionShapeOffsetTransform = shapeOffsetTransform;
                    if(doNotify){
                        collisionShape->notifyUpdate(sgUpdate.withAction(SgUpdate::Added | SgUpdate::Removed));
                    }
                }
            } else if(!collisionShape->empty()){
                collisionShapeOffsetTransform = new SgPosTransform(shapeOffset);
                collisionShape->moveChildrenTo(collisionShapeOffsetTransform);
                collisionShape->addChild(collisionShapeOffsetTransform);
                if(doNotify){
                    collisionShape->notifyUpdate(sgUpdate.withAction(SgUpdate::Added | SgUpdate::Removed));
                }
            }
        }
    } else if(shapeOffsetTransform || collisionShapeOffsetTransform){
        cancelOverwritingShapeOffset(link, doNotify);
    }
}


void LinkOverwriteItem::Impl::cancelOverwritingShapeOffset(Link* link, bool doNotify)
{
    if(shapeOffsetTransform){
        auto shape = link->visualShape();
        shape->contains(shapeOffsetTransform);
        shape->removeChild(shapeOffsetTransform);
        shapeOffsetTransform->copyChildrenTo(shape);
        shapeOffsetTransform.reset();
        if(doNotify){
            shape->notifyUpdate(sgUpdate.withAction(SgUpdate::Added | SgUpdate::Removed));
        }
    }
    if(collisionShapeOffsetTransform){
        auto shape = link->collisionShape();
        shape->contains(collisionShapeOffsetTransform);
        shape->removeChild(collisionShapeOffsetTransform);
        collisionShapeOffsetTransform->moveChildrenTo(shape);
        collisionShapeOffsetTransform.reset();
        if(doNotify){
            shape->notifyUpdate(sgUpdate.withAction(SgUpdate::Added | SgUpdate::Removed));
        }
    }
}


void LinkOverwriteItem::Impl::overwriteShapeColor(Link* link, bool doNotify)
{
    if(overwriteElementSet & ShapeColor){
        for(auto& kv : materialMap){
            kv.second.isUpdated = false;
        }
        if(!materialForNoMaterialShapes){
            materialForNoMaterialShapes = new SgMaterial;
        }
        materialForNoMaterialShapes->setDiffuseColor(shapeColor);
        bool materialForNoMaterialShapesAdded = false;

        link->shape()->traverseNodes(
            [this, &materialForNoMaterialShapesAdded](SgNode* node) -> SgObject::TraverseStatus {
                if(auto shape = dynamic_cast<SgShape*>(node)){
                    if(noMaterialShapes.find(shape) == noMaterialShapes.end()){
                        if(auto material = shape->material()){
                            auto& info = materialMap[material];
                            if(!info.isUpdated){
                                if(!info.orgMaterialClone){
                                    info.orgMaterialClone = static_cast<SgMaterial*>(material->clone());
                                }
                                material->setDiffuseColor(shapeColor);
                                if(!material->emissiveColor().isZero()){
                                    material->setEmissiveColor(shapeColor);
                                }
                                info.isUpdated = true;
                            }
                        } else {
                            shape->setMaterial(materialForNoMaterialShapes);
                            noMaterialShapes.insert(shape);
                            materialForNoMaterialShapesAdded = true;
                        }
                    }
                }
                return SgObject::Continue;
            });

        if(doNotify){
            sgUpdate.setAction(SgUpdate::Modified);
            for(auto& kv : materialMap){
                kv.first->notifyUpdate(sgUpdate);
            }
            if(materialForNoMaterialShapesAdded){
                sgUpdate.addAction(SgUpdate::Added);
                materialForNoMaterialShapes->notifyUpdate(sgUpdate);
            }
        }

    } else if(!materialMap.empty() || materialForNoMaterialShapes){
        cancelOverwritingShapeColor(link, doNotify);
    }
}


void LinkOverwriteItem::Impl::cancelOverwritingShapeColor(Link* link, bool doNotify)
{
    restoreOriginalShapeMaterials(link, doNotify);
    materialMap.clear();
    noMaterialShapes.clear();
    materialForNoMaterialShapes.reset();
    overwriteElementSet &= ~ShapeColor;
}


void LinkOverwriteItem::Impl::restoreOriginalShapeMaterials(Link* link, bool doNotify)
{
    for(auto& kv : materialMap){
        auto& material = kv.first;
        auto& info = kv.second;
        if(info.orgMaterialClone){
            material->copyMaterialPropertiesFrom(info.orgMaterialClone);
        }
    }
    for(auto& shape : noMaterialShapes){
        shape->setMaterial(nullptr);
    }
    if(doNotify){
        notifyMaterialUpdates(SgUpdate::Removed);
    }
}


void LinkOverwriteItem::Impl::restoreOverwritingShapeMaterials(Link* link, bool doNotify)
{
    for(auto& kv : materialMap){
        auto& material = kv.first;
        material->setDiffuseColor(shapeColor);
        if(!material->emissiveColor().isZero()){
            material->setEmissiveColor(shapeColor);
        }
    }
    for(auto& shape : noMaterialShapes){
        shape->setMaterial(materialForNoMaterialShapes);
    }
    if(doNotify){
        notifyMaterialUpdates(SgUpdate::Added);
    }
}


void LinkOverwriteItem::Impl::notifyMaterialUpdates(int noMaterialShapeAction)
{
    sgUpdate.setAction(SgUpdate::Modified);
    for(auto& kv : materialMap){
        auto& material = kv.first;
        material->notifyUpdate(sgUpdate);
    }
    if(!noMaterialShapes.empty()){
        sgUpdate.addAction(noMaterialShapeAction);
        for(auto& shape : noMaterialShapes){
            shape->notifyUpdate(sgUpdate);
        }
    }
}


bool LinkOverwriteItem::Impl::addNewLink(Body* body)
{
    bool added = false;
    
    Link* parentLink = nullptr;

    if(!additionalLinkParentName.empty()){
        parentLink = body->link(additionalLinkParentName);
    }
    if(!parentLink){
        if(auto parentLinkOverwriteItem = self->parentItem<LinkOverwriteItem>()){
            if(parentLinkOverwriteItem->isOverwriting()){
                parentLink = parentLinkOverwriteItem->impl->targetLink;
            }
        }
    }
    if(parentLink){
        parentLink->appendChild(additionalLink);
        added = true;
    }

    return added;
}


void LinkOverwriteItem::releaseOverwriteTarget()
{
    impl->releaseOverwriteTarget();
}


void LinkOverwriteItem::Impl::releaseOverwriteTarget()
{
    if(offsetLocation){
        offsetLocation->expire();
    }
    offsetLocation.reset();

    bodyItemConnection.disconnect();

    if(targetLink){
        self->bodyOverwrite()->unregisterLinkOverwriteItem(self);
        targetLink.reset();
    }
    referenceLink.reset();
    additionalLink.reset();
    originalLinkClone.reset();
    isRootLink = false;
    shapeOffsetTransform.reset();
    collisionShapeOffsetTransform.reset();
    materialMap.clear();
    noMaterialShapes.clear();
    materialForNoMaterialShapes.reset();
}


void LinkOverwriteItem::Impl::cancelOverwriting()
{
    bool updated = false;
    
    if(targetLink){
        if(auto body = targetLink->body()){
            if(referenceLink){
                if(originalLinkClone){
                    copyOverwriteLinkElements(originalLinkClone, targetLink);
                }
                cancelOverwritingShapeElements(targetLink);
                updated = true;
            } else if(additionalLink){
                if(auto parent = targetLink->parent()){
                    parent->removeChild(targetLink);
                    updated = true;
                }
            }
            if(updated){
                body->updateLinkTree();
            }
        }
    }

    releaseOverwriteTarget();

    if(updated){
        if(auto bodyItem = self->bodyItem()){
            bodyItem->notifyModelUpdate(
                BodyItem::LinkSetUpdate | BodyItem::DeviceSetUpdate | BodyItem::ShapeUpdate);
        }
    }
}


LocationProxyPtr LinkOverwriteItem::getLocationProxy()
{
    if(!impl->offsetLocation){
        impl->offsetLocation = new OffsetLocation(impl);
    }
    return impl->offsetLocation;
}


void LinkOverwriteItem::setLocationTargetType(int type)
{
    if(type != impl->locationTargetType){
        impl->locationTargetType = type;
        if(impl->offsetLocation){
            impl->offsetLocation->notifyAttributeChange();
            impl->offsetLocation->sigLocationChanged_();
        }
    }
}


int LinkOverwriteItem::locationTargetType() const
{
    return impl->locationTargetType;
}


SgNode* LinkOverwriteItem::getScene()
{
    if(!impl->originMarker){
        impl->updateLinkOriginMarker();
    }
    return impl->originMarker;
}


void LinkOverwriteItem::Impl::updateLinkOriginMarker()
{
    if(!originMarker){
        originMarker = new PositionDragger(
            PositionDragger::TranslationAxes, PositionDragger::PositiveOnlyHandle);
        originMarker->setDragEnabled(false);
        originMarker->setOverlayMode(true);
        originMarker->setPixelSize(96, 3);
        originMarker->setDisplayMode(PositionDragger::DisplayInEditMode);
    }
    if(targetLink){
        originMarker->setPosition(targetLink->T());
        originMarker->notifyUpdate();
    }
}


void LinkOverwriteItem::doPutProperties(PutPropertyFunction& putProperty)
{
    Item::doPutProperties(putProperty);
}


bool LinkOverwriteItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool LinkOverwriteItem::Impl::store(Archive& archive)
{
    auto link = self->sourceLink();
    if(!link){
        return false;
    }

    if(isRootLink){
        archive.write("is_root", true);
    }

    bool isAddingLink = self->isAddingLink();
    if(isAddingLink){
        archive.write("is_additional", true);
        if(auto parentLink = link->parent()){
            archive.write("parent", parentLink->name(), DOUBLE_QUOTED);
        } else if(!additionalLinkParentName.empty()){
            archive.write("parent", additionalLinkParentName, DOUBLE_QUOTED);
        }
    }
    
    archive.setFloatingNumberFormat("%.9g");
    
    if(overwriteElementSet & OffsetPosition){
        // The "translation" value is always written so that the restore function can
        // know that the OffsetPosition is an element to overwrite.
        write(archive, "translation", link->offsetTranslation());
        AngleAxis aa(link->offsetRotation());
        if(aa.angle() != 0.0){
            writeDegreeAngleAxis(archive, "rotation", aa);
        }
    }
    if(overwriteElementSet & JointType){
        archive.write("joint_type", link->jointTypeSymbol());
    }
    if(overwriteElementSet & JointAxis){
        write(archive, "joint_axis", link->jointAxis());
    }
    if(overwriteElementSet & JointId){
        archive.write("joint_id", link->jointId());
    }
    if(overwriteElementSet & JointName){
        archive.write("joint_name", link->jointName(), DOUBLE_QUOTED);
    }
    if(overwriteElementSet & JointRange){
        StdBodyWriter::writeJointDisplacementRange(&archive, link, true);
    }
    if(overwriteElementSet & JointVelocityRange){
        StdBodyWriter::writeJointVelocityRange(&archive, link, true);
    }
    if(overwriteElementSet & JointEffortRange){
        StdBodyWriter::writeJointEffortRange(&archive, link, true);
    }

    if(overwriteElementSet & ShapeOffset){
        write(archive, "shape_translation", shapeOffset.translation());
        AngleAxis aa(shapeOffset.linear());
        if(aa.angle() != 0.0){
            writeDegreeAngleAxis(archive, "shape_rotation", aa);
        }
    }
    
    if(overwriteElementSet & ShapeColor){
        write(archive, "shape_color", shapeColor);
    }
    
    if(overwriteElementSet & Shape){
        if(!sceneWriter){
            sceneWriter = sharedSceneWriter.lock();
            if(!sceneWriter){
                sceneWriter = make_shared<StdSceneWriter>();
                sceneWriter->setTopGroupNodeSkippingEnabled(true);
                sceneWriter->setExtModelFileMode(StdSceneWriter::LinkToOriginalModelFiles);
                sharedSceneWriter = sceneWriter;
            }
        }
        sceneWriter->setFilePathVariableProcessor(archive.filePathVariableProcessor());

        if(overwriteElementSet & ShapeColor){
            restoreOriginalShapeMaterials(link, false);
        }
        bool hasDedicatedCollisionShape = (visualShape != collisionShape);
        if(visualShape){
            auto visualShapeArchive = sceneWriter->writeScene(visualShape);
            if(!hasDedicatedCollisionShape){
                archive.insert("shape", visualShapeArchive);
            } else {
                archive.insert("visual_shape", visualShapeArchive);
            }
        }
        if(collisionShape && hasDedicatedCollisionShape){
            if(auto collisionShapeArchive = sceneWriter->writeScene(collisionShape)){
                archive.insert("collision_shape", collisionShapeArchive);
            }
        }
        if(overwriteElementSet & ShapeColor){
            restoreOverwritingShapeMaterials(link, false);
        }
    }

    return true;
}


shared_ptr<StdSceneReader> LinkOverwriteItem::Impl::ensureSceneReader(const Archive& archive)
{
    if(!sceneReader){
        sceneReader = sharedSceneReader.lock();
        if(!sceneReader){
            sceneReader = make_shared<StdSceneReader>();
            sceneReader->setGroupOptimizationEnabled(false); // temporary
            sceneReader->setAngleUnit(StdSceneReader::DEGREE);
            sharedSceneReader = sceneReader;
        }
    }
    sceneReader->setFilePathVariableProcessor(archive.filePathVariableProcessor());
    return sceneReader;
}


bool LinkOverwriteItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool LinkOverwriteItem::Impl::restore(const Archive& archive)
{
    // For backward compatibility
    auto node = archive.find("overwrite_shape");
    if(node->isValid()){
        return restoreShapeWrittenInOldFormat(archive, node);
    }

    if(self->name().empty()){
        return false;
    }

    LinkPtr link = new Link;
    link->setName(self->name());

    overwriteElementSet = NoElement;

    if(!archive.get("is_additional", false)){
        self->setReferenceLink(link, archive.get("is_root", false));
    } else {
        string parentName;
        archive.read("parent", parentName);
        self->setAdditionalLink(link, parentName);
    }
    Vector3 p;
    if(read(archive, "translation", p)){
        link->setOffsetTranslation(p);
        AngleAxis aa;
        if(readDegreeAngleAxis(archive, "rotation", aa)){
            link->setOffsetRotation(aa);
        }
        overwriteElementSet |= OffsetPosition;
    }
    string symbol;
    if(archive.read("joint_type", symbol)){
        if(symbol == "revolute"){
            link->setJointType(Link::RevoluteJoint);
        } else if(symbol == "prismatic"){
            link->setJointType(Link::PrismaticJoint);
        } else if(symbol == "free"){
            link->setJointType(Link::FreeJoint);
        } else if(symbol == "fixed"){
            link->setJointType(Link::FixedJoint);
        } else {
            archive.throwException(formatR(_("Illegal jointType value \"{0}\""), symbol));
        }
        overwriteElementSet |= JointType;
    }
    Vector3 a;
    if(read(archive, "joint_axis", a)){
        link->setJointAxis(a);
        overwriteElementSet |= JointAxis;
    }
    int id;
    if(archive.read("joint_id", id)){
        link->setJointId(id);
        overwriteElementSet |= JointId;
    }
    if(archive.read("joint_name", symbol)){
        if(symbol.empty()){
            link->resetJointSpecificName();
        } else {
            link->setJointName(symbol);
        }
        overwriteElementSet |= JointName;
    }
    if(StdBodyLoader::readJointDisplacementRange(&archive, link)){
        overwriteElementSet |= JointRange;
    }
    if(StdBodyLoader::readJointVelocityRange(&archive, link)){
        overwriteElementSet |= JointVelocityRange;
    }
    if(StdBodyLoader::readJointEffortRange(&archive, link)){
        overwriteElementSet |= JointEffortRange;
    }

    if(read(archive, "shape_translation", p)){
        Isometry3 T;
        T.translation() = p;
        AngleAxis aa;
        if(readDegreeAngleAxis(archive, "shape_rotation", aa)){
            T.linear() = aa.toRotationMatrix();
        } else {
            T.linear().setIdentity();
        }
        self->setShapeOffset(T);
    }
    
    Vector3f c;
    if(read(archive, "shape_color", c)){
        shapeColor = c;
        overwriteElementSet |= ShapeColor;
    }

    auto shapeArchive = archive.find("shape");
    if(shapeArchive->isValid()){
        if(auto shape = ensureSceneReader(archive)->readScene(shapeArchive)){
            self->setShape(shape);
        }
    }
    auto visualShapeArchive = archive.find("visual_shape");
    if(visualShapeArchive->isValid()){
        if(auto visualShape = ensureSceneReader(archive)->readScene(visualShapeArchive)){
            self->setVisualShape(visualShape);
        }
    }
    auto collisionShapeArchive = archive.find("collision_shape");
    if(collisionShapeArchive->isValid()){
        if(auto collisionShape = ensureSceneReader(archive)->readScene(collisionShapeArchive)){
            self->setCollisionShape(collisionShape);
        }
    }

    return true;
}


bool LinkOverwriteItem::Impl::restoreShapeWrittenInOldFormat(const Archive& archive, ValueNode* shapeArchive)
{
    auto bodyItem = archive.currentParentItem()->findOwnerItem<BodyItem>(true);
    if(!bodyItem){
        MessageView::instance()->putln(
            formatR(_("The target body item of \"{0}\" is not found."), self->displayName()),
            MessageView::Error);
        return false;
    }

    SgNodePtr shape = ensureSceneReader(archive)->readScene(shapeArchive);
    if(shape){
        setReferenceLinkToRestoreShapeWritteinInOldFormat(bodyItem->body()->rootLink(), shape);
        return true;
    }

    return false;
}


bool checkHintsInMetadata(SgObject* object, bool& out_isNotMeter, bool& out_isYUpperAxis)
{
    auto metadata = object->uriMetadata();
    if(!metadata){
        return false;
    }
    string symbol;
    if(metadata->read("length_unit", symbol)){
        if(symbol != "meter"){
            out_isNotMeter = true;
        }
    }
    if(metadata->read("upper_axis", symbol)){
        if(symbol == "Y"){
            out_isYUpperAxis = true;
        }
    }
    return true;
}


void LinkOverwriteItem::Impl::setReferenceLinkToRestoreShapeWritteinInOldFormat(Link* orgLink, SgNode* newShape)
{
    SgPosTransformPtr shapeOffsetNode = nullptr;
    vector<SgNodePath> shapeNodePaths;
    SceneNodeExtractor nodeExtractor;

    if(newShape){
        shapeOffsetNode = dynamic_cast<SgPosTransform*>(newShape);
        if(!shapeOffsetNode){
            shapeOffsetNode = new SgPosTransform;
            shapeOffsetNode->addChild(newShape);
        }
        shapeNodePaths = nodeExtractor.extractNodes<SgShape>(newShape, true);
    }

    CloneMap cloneMap;
    SgObject::setNonNodeCloning(cloneMap, false);
    SgNodePtr shapeClone = cloneMap.getClone(orgLink->shape());
    vector<SgNodePath> existingShapeNodePaths = nodeExtractor.extractNodes<SgShape>(shapeClone, false);
    bool doRemoveScalingForLengthUnitConversion = false;
    bool doRemoveRotationForUpperAxisConversion = false;

    if(existingShapeNodePaths.empty()){
        if(!newShape){
            shapeOffsetNode = new SgPosTransform;
        }
    } else {
        if(!newShape){
            shapeOffsetNode = extractOrInsertOffsetTransform(existingShapeNodePaths);
        } else {
            int n = std::min(shapeNodePaths.size(), existingShapeNodePaths.size());
            for(int i=0; i < n; ++i){
                auto shapeNode = static_cast<SgShape*>(shapeNodePaths[i].back().get());
                auto existingShapeNode = static_cast<SgShape*>(existingShapeNodePaths[i].back().get());
                if(!shapeNode->mesh()){
                    auto existingMesh = existingShapeNode->mesh();
                    shapeNode->setMesh(existingMesh);
                    
                    if(!checkHintsInMetadata(
                           existingShapeNode, doRemoveScalingForLengthUnitConversion, doRemoveRotationForUpperAxisConversion)){
                        checkHintsInMetadata(
                            existingMesh, doRemoveScalingForLengthUnitConversion, doRemoveRotationForUpperAxisConversion);
                    }
                }
                if(!shapeNode->material()){
                    shapeNode->setMaterial(existingShapeNode->material());
                }
            }
        }
    }

    Isometry3 T = shapeOffsetNode->position();
    if(doRemoveRotationForUpperAxisConversion){
        Matrix3 R;
        R << 0, 0, 1,
             1, 0, 0,
             0, 1, 0;
        T.linear() = T.linear() * R.transpose();
    }
    if(!T.matrix().isIdentity()){
        self->setShapeOffset(T);
    }
    if(!shapeOffsetNode->empty()){
        SgNode* node = shapeOffsetNode->child(0);
        if(doRemoveScalingForLengthUnitConversion){
            if(auto scale = dynamic_cast<SgScaleTransform*>(node)){
                node = scale->child(0);
            }
        }
        self->setShape(node);
    }

    LinkPtr link = new Link;
    link->setName(orgLink->name());
    self->setReferenceLink(link);
}


SgPosTransform* LinkOverwriteItem::Impl::extractOrInsertOffsetTransform(vector<SgNodePath>& nodePaths)
{
    SgNode* firstTopNode = nodePaths.front().front();
    SgPosTransform* offset = dynamic_cast<SgPosTransform*>(firstTopNode);
    if(offset){
        for(size_t i=1; i < nodePaths.size(); ++i){
            auto& nodePath = nodePaths[i];
            if(nodePath.front() != firstTopNode){
                offset = nullptr;
                break;
            }
        }
    }
    if(!offset){
        offset = new SgPosTransform;
        for(auto& nodePath : nodePaths){
            offset->addChildOnce(nodePath.front());
            nodePath.insert(nodePath.begin(), offset);
        }
    }
    return offset;
}


OffsetLocation::OffsetLocation(LinkOverwriteItem::Impl* impl)
    : LocationProxy(LocationProxy::OffsetLocation),
      impl(impl)
{
    setLocked(true);
}


bool OffsetLocation::isLinkOffsetLocation() const
{
    return impl->locationTargetType == LinkOverwriteItem::LinkOffsetLocation;
}


bool OffsetLocation::isShapeOffsetLocation() const
{
    return impl->locationTargetType == LinkOverwriteItem::ShapeOffsetLocation;
}


std::string OffsetLocation::getName() const
{
    auto name = LocationProxy::getName();
    if(isShapeOffsetLocation()){
        return formatR(_("{0} Shape"), name);
    }
    return name;
}


Isometry3 OffsetLocation::getLocation() const
{
    if(impl->targetLink){
        if(isLinkOffsetLocation()){
            return impl->targetLink->offsetPosition();
        } else if(isShapeOffsetLocation()){
            return impl->shapeOffset;
        }
    }
    return Isometry3::Identity();
}


bool OffsetLocation::setLocation(const Isometry3& T)
{
    bool updated = false;
    if(auto sourceLink = impl->self->sourceLink()){
        if(isLinkOffsetLocation()){
            sourceLink->setOffsetPosition(T);
            updated = true;
        } else if(isShapeOffsetLocation()){
            impl->self->setShapeOffset(T, true);
            updated = true;
        }
    }
    if(updated){
        updated = impl->self->updateOverwriting();
    }
    return updated;
}


Item* OffsetLocation::getCorrespondingItem()
{
    return impl->self;
}


LocationProxyPtr OffsetLocation::getParentLocationProxy() const
{
    if(impl->targetLink){
        if(isLinkOffsetLocation()){
            if(!parentLinkLocation){
                if(auto parentLink = impl->targetLink->parent()){
                    parentLinkLocation = impl->self->bodyItem()->createLinkLocationProxy(parentLink);
                }
            }
            return parentLinkLocation;
            
        } else if(isShapeOffsetLocation()){
            if(!linkLocation){
                linkLocation = impl->self->bodyItem()->createLinkLocationProxy(impl->targetLink);
            }
            return linkLocation;
        }
    }
    return nullptr;
}


SignalProxy<void()> OffsetLocation::sigLocationChanged()
{
    return sigLocationChanged_;
}
