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

class LinkOffsetLocation : public LocationProxy
{
public:
    Signal<void()> sigLocationChanged_;

};

typedef ref_ptr<LinkOffsetLocation> LinkOffsetLocationPtr;

class LinkShapeLocation : public LocationProxy
{
public:
    LinkOverwriteItem::Impl* impl;
    Signal<void()> sigLocationChanged_;

    LinkShapeLocation(LinkOverwriteItem::Impl* impl);
    virtual Isometry3 getLocation() const override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual Item* getCorrespondingItem() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

typedef ref_ptr<LinkShapeLocation> LinkShapeLocationPtr;

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
    int modifiedLinkElementSet;

    PositionDraggerPtr originMarker;
    LinkOffsetLocationPtr linkOffsetLocation;
    LinkShapeLocationPtr linkShapeLocation;
    ScopedConnection bodyItemConnection;

    shared_ptr<StdSceneReader> sceneReader;
    shared_ptr<StdSceneWriter> sceneWriter;

    Impl(LinkOverwriteItem* self);
    Impl(LinkOverwriteItem* self, const Impl& org);
    bool updateOverwriting(BodyItem* bodyItem);
    void overwriteExistingLink(Link* existingLink);
    void copyTargetElements(Link* srcLink, Link* destLink, int elementSet);
    bool addNewLink(Body* body);
    void clearOverwriting();
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
}


LinkOverwriteItem::LinkOverwriteItem(const LinkOverwriteItem& org)
    : BodyElementOverwriteItem(org)
{
    impl = new Impl(this, *org.impl);
}


LinkOverwriteItem::Impl::Impl(LinkOverwriteItem* self, const Impl& org)
    : self(self),
      targetElementSet(org.targetElementSet),
      additionalLinkParentName(org.additionalLinkParentName)
{
    if(org.referenceLink){
        referenceLink = org.referenceLink->clone();
    }
    if(org.additionalLink){
        additionalLink = org.additionalLink->clone();
    }
    if(org.originalLinkClone){
        originalLinkClone = org.originalLinkClone->clone();
    }
}


LinkOverwriteItem::~LinkOverwriteItem()
{
    delete impl;
}


Item* LinkOverwriteItem::doDuplicate(Item* duplicatedParentItem) const
{
    return new LinkOverwriteItem(*this);
}


bool LinkOverwriteItem::onCheckNewOverwritePosition(bool isManualOperation)
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
    impl->clearOverwriting();
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
            bodyItem_->notifyModelUpdate(BodyItem::LinkSetUpdate);
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


bool LinkOverwriteItem::setReferenceLink(Link* referenceLink)
{
    if(isOverwriting()){
        return false;
    }
    setName(referenceLink->name());
    impl->referenceLink = referenceLink;
    impl->additionalLink.reset();
    impl->additionalLinkParentName.clear();
    return true;
}


Link* LinkOverwriteItem::referenceLink()
{
    return impl->referenceLink;
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
            newTargetLink = body->link(referenceLink->name());
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
        if((additionalLink && newTargetLink) ||
           (targetElementSet & (JointType | JointId | JointName))){
            body->updateLinkTree();
        }

        bodyItem->notifyModelUpdate(
            BodyItem::LinkSetUpdate | BodyItem::DeviceSetUpdate | BodyItem::ShapeUpdate);

        if(targetElementSet & (OffsetPosition | JointType | JointAxis)){
            bodyItem->notifyKinematicStateChange(true);
        }

        // todo: Execute the following code only when the target body item is changed
        bodyItemConnection =
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ updateLinkOriginMarker(); });
    
        if(originMarker){
            updateLinkOriginMarker();
        }

        if(targetElementSet & Shape){
            if(linkShapeLocation){
                linkShapeLocation->sigLocationChanged_();
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
        if(auto parentLinkOverwriteItem = dynamic_cast<LinkOverwriteItem*>(self->parentItem())){
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


void LinkOverwriteItem::clearOverwriting()
{
    impl->clearOverwriting();
}


void LinkOverwriteItem::Impl::clearOverwriting()
{
    bodyItemConnection.disconnect();
    
    if(targetLink){
        if(auto body = targetLink->body()){
            bool updated = false;
            if(originalLinkClone){
                copyTargetElements(originalLinkClone, targetLink, modifiedLinkElementSet);
                updated = true;
            } else if(auto parent = targetLink->parent()){
                parent->removeChild(targetLink);
                updated = true;
            }
            if(updated){
                body->updateLinkTree();
            }
        }
        targetLink.reset();
        self->bodyOverwrite()->unregisterLinkOverwriteItem(self);
    }

    originalLinkClone.reset();

    if(linkShapeLocation){
        linkShapeLocation->expire();
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
    if(!impl->linkShapeLocation){
        impl->linkShapeLocation = new LinkShapeLocation(impl);
    }
    return impl->linkShapeLocation;
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
        self->setReferenceLink(link);
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


LinkShapeLocation::LinkShapeLocation(LinkOverwriteItem::Impl* impl)
    : LocationProxy(OffsetLocation),
      impl(impl)
{
    setEditable(false);
}
    

Isometry3 LinkShapeLocation::getLocation() const
{
    if(auto transform = impl->getShapeOffsetTransform(impl->targetLink)){
        return transform->T();
    }
    return Isometry3::Identity();
}


bool LinkShapeLocation::setLocation(const Isometry3& T)
{
    if(auto transform = impl->getShapeOffsetTransform(impl->self->sourceLink())){
        transform->setPosition(T);
        transform->notifyUpdate();
        return impl->self->updateOverwriting();
    }
    return false;
}


Item* LinkShapeLocation::getCorrespondingItem()
{
    return impl->self;
}


LocationProxyPtr LinkShapeLocation::getParentLocationProxy() const
{
    return impl->self->bodyItem()->getLocationProxy();
}


SignalProxy<void()> LinkShapeLocation::sigLocationChanged()
{
    return sigLocationChanged_;
}
