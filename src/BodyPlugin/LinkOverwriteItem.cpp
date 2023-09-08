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
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

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
    int targetElementSet;
    LinkPtr targetLink;
    LinkPtr referenceLink;
    LinkPtr additionalLink;
    string additionalLinkParentName;
    LinkPtr originalLinkClone;
    bool isRootLink;

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
    void copyTargetElements(Link* srcLink, Link* destLink, int elementSet);
    bool addNewLink(Body* body);
    void releaseOverwriteTarget();
    void cancelOverwriting();
    SgPosTransform* getShapeOffsetTransform(Link* link);
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
    targetElementSet = NoElement;
    isRootLink = false;
    locationTargetType = LinkOffsetLocation;
}


LinkOverwriteItem::LinkOverwriteItem(const LinkOverwriteItem& org, CloneMap* cloneMap)
    : BodyElementOverwriteItem(org, cloneMap)
{
    impl = new Impl(this, *org.impl, cloneMap);
}


LinkOverwriteItem::Impl::Impl(LinkOverwriteItem* self, const Impl& org, CloneMap* cloneMap)
    : self(self),
      targetElementSet(org.targetElementSet),
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
    if(org.originalLinkClone){
        originalLinkClone = CloneMap::getClone(org.originalLinkClone, cloneMap);
    }
    isRootLink = org.isRootLink;
    locationTargetType = org.locationTargetType;
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


void LinkOverwriteItem::setTargetElementSet(int elementSet)
{
    impl->targetElementSet = elementSet;
}


void LinkOverwriteItem::addTargetElement(int element)
{
    impl->targetElementSet |= element;
}


int LinkOverwriteItem::targetElementSet() const
{
    return impl->targetElementSet;
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

    if(!targetLink){
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
                }
                if(updated){
                    targetLink = newTargetLink;
                } else {
                    addon->unregisterLinkOverwriteItem(self);
                }
            }
        }
    } else {
        if(referenceLink){
            overwriteExistingLink(targetLink);
            updated = true;
        } else if(additionalLink){
            updated = true;
        }
    }

    if(updated){
        bool isKinematicStateChangeNotificationRequested = false;
        
        if((additionalLink && newTargetLink) ||
           (targetElementSet & (JointType | JointId | JointName))){
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
            if(targetElementSet & (OffsetPosition | JointType | JointAxis)){
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
            if(((targetElementSet & OffsetPosition) && (locationTargetType == LinkOffsetLocation)) ||
               ((targetElementSet & Shape) && (locationTargetType == ShapeOffsetLocation))){
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
    copyTargetElements(referenceLink, existingLink, targetElementSet);
}


void LinkOverwriteItem::Impl::copyTargetElements(Link* srcLink, Link* destLink, int elementSet)
{
    if(isRootLink){
        destLink->setName(srcLink->name());
    }
    if(elementSet & OffsetPosition){
        destLink->setOffsetPosition(srcLink->offsetPosition());
    }
    if(elementSet & JointType){
        destLink->setJointType(srcLink->jointType());
    }
    if(elementSet & JointAxis){
        destLink->setJointAxis(srcLink->jointAxis());
    }
    if(elementSet & JointId){
        destLink->setJointId(srcLink->jointId());
    }
    if(elementSet & JointName){
        destLink->setJointName(srcLink->jointName());
    }
    if(elementSet & JointRange){
        destLink->setJointRange(srcLink->q_lower(), srcLink->q_upper());
    }
    if(elementSet & JointVelocityRange){
        destLink->setJointVelocityRange(srcLink->dq_lower(), srcLink->dq_upper());
    }
    if(elementSet & JointEffortRange){
        destLink->setJointEffortRange(srcLink->u_lower(), srcLink->u_upper());
    }
    if(elementSet & Mass){
        destLink->setMass(srcLink->mass());
    }
    if(elementSet & Inertia){
        destLink->setInertia(srcLink->I());
    }
    if(elementSet & CenterOfMass){
        destLink->setCenterOfMass(srcLink->centerOfMass());
    }
    if(elementSet & Material){
        destLink->setMaterial(srcLink->materialId());
    }
    if(elementSet & Shape){
        SgUpdate update;
        destLink->clearShapeNodes(update);
        for(auto& node : *srcLink->visualShape()){
            destLink->addVisualShapeNode(node, update);
        }
        for(auto& node : *srcLink->collisionShape()){
            destLink->addCollisionShapeNode(node, update);
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
}


void LinkOverwriteItem::Impl::cancelOverwriting()
{
    bool updated = false;
    
    if(targetLink){
        if(auto body = targetLink->body()){
            if(originalLinkClone){
                copyTargetElements(originalLinkClone, targetLink, targetElementSet);
                updated = true;
            } else if(auto parent = targetLink->parent()){
                parent->removeChild(targetLink);
                updated = true;
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


SgPosTransform* LinkOverwriteItem::Impl::getShapeOffsetTransform(Link* link)
{
    if(link){
        auto shape = link->shape();
        if(shape->numChildren() == 1){
            if(auto transform = dynamic_cast<SgPosTransform*>(shape->child(0))){
                return transform;
            }
        }
    }
    return nullptr;
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

    if(self->isAddingLink()){
        archive.write("is_additional", true);
        if(auto parentLink = link->parent()){
            archive.write("parent", parentLink->name(), DOUBLE_QUOTED);
        } else if(!additionalLinkParentName.empty()){
            archive.write("parent", additionalLinkParentName, DOUBLE_QUOTED);
        }
    }
    
    archive.setFloatingNumberFormat("%.9g");
    
    if(targetElementSet & OffsetPosition){
        // The "translation" value is always written so that the restore function can
        // know that the OffsetPosition is an element to overwrite.
        write(archive, "translation", link->offsetTranslation());
        AngleAxis aa(link->offsetRotation());
        if(aa.angle() != 0.0){
            writeDegreeAngleAxis(archive, "rotation", aa);
        }
    }
    if(targetElementSet & JointType){
        archive.write("joint_type", link->jointTypeSymbol());
    }
    if(targetElementSet & JointAxis){
        write(archive, "joint_axis", link->jointAxis());
    }
    if(targetElementSet & JointId){
        archive.write("joint_id", link->jointId());
    }
    if(targetElementSet & JointName){
        archive.write("joint_name", link->jointName(), DOUBLE_QUOTED);
    }
    if(targetElementSet & JointRange){
        StdBodyWriter::writeJointDisplacementRange(&archive, link, true);
    }
    if(targetElementSet & JointVelocityRange){
        StdBodyWriter::writeJointVelocityRange(&archive, link, true);
    }
    if(targetElementSet & JointEffortRange){
        StdBodyWriter::writeJointEffortRange(&archive, link, true);
    }

    if(targetElementSet & Shape){
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
        if(auto sceneArchive = sceneWriter->writeScene(link->shape())){
            archive.insert("shape", sceneArchive);
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

    int elementSet = NoElement;

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
        elementSet |= OffsetPosition;
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
            archive.throwException(format(_("Illegal jointType value \"{0}\""), symbol));
        }
        elementSet |= JointType;
    }
    Vector3 a;
    if(read(archive, "joint_axis", a)){
        link->setJointAxis(a);
        elementSet |= JointAxis;
    }
    int id;
    if(archive.read("joint_id", id)){
        link->setJointId(id);
        elementSet |= JointId;
    }
    if(archive.read("joint_name", symbol)){
        if(symbol.empty()){
            link->resetJointSpecificName();
        } else {
            link->setJointName(symbol);
        }
        elementSet |= JointName;
    }
    if(StdBodyLoader::readJointDisplacementRange(&archive, link)){
        elementSet |= JointRange;
    }
    if(StdBodyLoader::readJointVelocityRange(&archive, link)){
        elementSet |= JointVelocityRange;
    }
    if(StdBodyLoader::readJointEffortRange(&archive, link)){
        elementSet |= JointEffortRange;
    }

    auto shapeArchive = archive.find("shape");
    if(shapeArchive->isValid()){
        if(auto shape = ensureSceneReader(archive)->readScene(shapeArchive)){
            link->addShapeNode(shape);
            elementSet |= Shape;
        }
    }

    if(elementSet){
        self->setTargetElementSet(elementSet);
    }

    return elementSet != NoElement;
}


bool LinkOverwriteItem::Impl::restoreShapeWrittenInOldFormat(const Archive& archive, ValueNode* shapeArchive)
{
    auto bodyItem = archive.currentParentItem()->findOwnerItem<BodyItem>(true);
    if(!bodyItem){
        MessageView::instance()->putln(
            format(_("The target body item of \"{0}\" is not found."), self->displayName()),
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


void LinkOverwriteItem::Impl::setReferenceLinkToRestoreShapeWritteinInOldFormat(Link* orgLink, SgNode* newShape)
{
    SgPosTransformPtr shapeOffset = nullptr;
    vector<SgNodePath> shapeNodePaths;
    SceneNodeExtractor nodeExtractor;

    if(newShape){
        shapeOffset = dynamic_cast<SgPosTransform*>(newShape);
        if(!shapeOffset){
            shapeOffset = new SgPosTransform;
            shapeOffset->addChild(newShape);
        }
        shapeNodePaths = nodeExtractor.extractNodes<SgShape>(newShape, true);
    }

    CloneMap cloneMap;
    SgObject::setNonNodeCloning(cloneMap, false);
    SgNodePtr shapeClone = cloneMap.getClone(orgLink->shape());
    vector<SgNodePath> existingShapeNodePaths = nodeExtractor.extractNodes<SgShape>(shapeClone, false);

    if(existingShapeNodePaths.empty()){
        if(!newShape){
            shapeOffset = new SgPosTransform;
        }
    } else {
        if(!newShape){
            shapeOffset = extractOrInsertOffsetTransform(existingShapeNodePaths);
        } else {
            int n = std::min(shapeNodePaths.size(), existingShapeNodePaths.size());
            for(int i=0; i < n; ++i){
                auto shapeNode = static_cast<SgShape*>(shapeNodePaths[i].back().get());
                auto existingShapeNode = static_cast<SgShape*>(existingShapeNodePaths[i].back().get());
                if(!shapeNode->mesh()){
                    shapeNode->setMesh(existingShapeNode->mesh());
                }
                if(!shapeNode->material()){
                    shapeNode->setMaterial(existingShapeNode->material());
                }
            }
        }
    }

    LinkPtr link = new Link;
    link->setName(orgLink->name());
    link->addShapeNode(shapeOffset, true);

    self->setReferenceLink(link);
    self->setTargetElementSet(Shape);
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
        return format(_("{0} Shape"), name);
    }
    return name;
}


Isometry3 OffsetLocation::getLocation() const
{
    if(impl->targetLink){
        if(isLinkOffsetLocation()){
            return impl->targetLink->offsetPosition();
        } else if(isShapeOffsetLocation()){
            if(auto transform = impl->getShapeOffsetTransform(impl->targetLink)){
                return transform->T();
            }
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
            if(auto transform = impl->getShapeOffsetTransform(sourceLink)){
                transform->setPosition(T);
                transform->notifyUpdate();
                updated = true;
            }
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
