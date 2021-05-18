#include "LinkShapeOverwriteItem.h"
#include "BodyOverwriteAddon.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/SceneGraph>
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
#include <cnoid/MessageView>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class LinkShapeLocation : public LocationProxy
{
public:
    LinkShapeOverwriteItem::Impl* impl;

    LinkShapeLocation(LinkShapeOverwriteItem::Impl* impl);
    virtual Isometry3 getLocation() const override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual Item* getCorrespondingItem() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

}

namespace cnoid {

class LinkShapeOverwriteItem::Impl
{
public:
    LinkShapeOverwriteItem* self;
    Link* link;
    SgPosTransformPtr offsetTransform;
    Signal<void()> sigOffsetChanged;
    SgShapePtr shapeNode;
    SgGroupPtr orgLinkShape;
    ref_ptr<LinkShapeLocation> linkShapeLocation;
    PositionDraggerPtr originMarker;
    ScopedConnection bodyItemConnection;
    
    Impl(LinkShapeOverwriteItem* self);
    Impl(LinkShapeOverwriteItem* self, const Impl& org);
    void clear();
    bool overwriteLinkShape(BodyItem* bodyItem, Link* link, SgNode* newShape, SgGroup* orgLinkShapeForDuplicated);
    void extractShapeNodes(SgNodePath& nodePath);
    void updateLinkOriginMarker();
};

}


void LinkShapeOverwriteItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<LinkShapeOverwriteItem, BodyElementOverwriteItem>(N_("LinkShapeOverwriteItem"));
    im.addAlias<LinkShapeOverwriteItem>("LinkShapeOverwriteItem", "BodyEdit");
}


LinkShapeOverwriteItem::LinkShapeOverwriteItem()
{
    impl = new Impl(this);
}


LinkShapeOverwriteItem::Impl::Impl(LinkShapeOverwriteItem* self)
    : self(self)
{
    offsetTransform = new SgPosTransform;
    link = nullptr;
}


LinkShapeOverwriteItem::LinkShapeOverwriteItem(const LinkShapeOverwriteItem& org)
    : BodyElementOverwriteItem(org)
{
    impl = new Impl(this, *org.impl);
}


LinkShapeOverwriteItem::Impl::Impl(LinkShapeOverwriteItem* self, const Impl& org)
    : Impl(self)
{

}


LinkShapeOverwriteItem::~LinkShapeOverwriteItem()
{
    delete impl;
}



Item* LinkShapeOverwriteItem::doDuplicate(Item* duplicatedParentItem) const
{
    LinkShapeOverwriteItemPtr duplicated = new LinkShapeOverwriteItem(*this);

    if(auto bodyItem = dynamic_cast<BodyItem*>(duplicatedParentItem)){
        auto link = bodyItem->body()->link(impl->link->index());
        if(!duplicated->impl->overwriteLinkShape(bodyItem, link, nullptr, impl->orgLinkShape)){
            duplicated.reset();
        }
    }
    
    return duplicated.retn();
}


void LinkShapeOverwriteItem::Impl::clear()
{
    self->setBodyItem(nullptr);
    link = nullptr;
    if(linkShapeLocation){
        linkShapeLocation->expire();
    }
}    


bool LinkShapeOverwriteItem::overwriteLinkShape(BodyItem* bodyItem, Link* link)
{
    return impl->overwriteLinkShape(bodyItem, link, nullptr, nullptr);
}


//! \Note The return value of BodyOverwriteAddon::checkIfSingleShapeBody must be true
bool LinkShapeOverwriteItem::Impl::overwriteLinkShape
(BodyItem* bodyItem, Link* link, SgNode* newShape, SgGroup* orgLinkShapeForDuplicated)
{
    if(self->bodyItem()){
        return false; // This item has been overwriting
    }
    auto bodyOverwrite = bodyItem->getAddon<BodyOverwriteAddon>();
    if(!bodyOverwrite->registerLinkShapeOverwriteItem(link, self)){
        return false;
    }

    self->setBodyItem(bodyItem);
    this->link = link;

    if(self->name().empty()){
        self->setName(format(_("Shape ({0})"), link->name()));
    }

    if(orgLinkShapeForDuplicated){
        orgLinkShape = orgLinkShapeForDuplicated;
    } else {
        if(!orgLinkShape){
            orgLinkShape = new SgGroup;
        } else {
            orgLinkShape->clearChildren();
        }
        link->shape()->copyChildrenTo(orgLinkShape);
    }
    
    SceneNodeExtractor nodeExtractor;
    SgNodePath newNodePath;
    if(newShape){
        newNodePath = nodeExtractor.extractNode<SgShape>(newShape, true);
    }

    CloneMap cloneMap;
    SgNodePath existingNodePath;
    if(newNodePath.empty()){
        // Clone the original scene nodes to avoid modifying them
        existingNodePath = nodeExtractor.extractNode<SgShape>(cloneMap.getClone(link->shape()), false);
    } else {
        existingNodePath = nodeExtractor.extractNode<SgShape>(link->shape(), false);
    }

    if(existingNodePath.empty()){
        if(newNodePath.empty()){
            offsetTransform->setPosition(Isometry3::Identity());
            shapeNode.reset();
        } else {
            extractShapeNodes(newNodePath);
        }
    } else {
        if(newNodePath.empty()){
            extractShapeNodes(existingNodePath);
        } else {
            extractShapeNodes(newNodePath);
            auto existingShapeNode = static_cast<SgShape*>(existingNodePath.back());
            if(!shapeNode->mesh()){
                shapeNode->setMesh(existingShapeNode->mesh());
            }
            if(!shapeNode->material()){
                shapeNode->setMaterial(cloneMap.getClone(existingShapeNode->material()));
            }
        }
    }

    link->clearShapeNodes();
    SgTmpUpdate update;
    link->addShapeNode(offsetTransform, update);

    bodyItemConnection =
        bodyItem->sigKinematicStateChanged().connect(
            [&](){ updateLinkOriginMarker(); });
    
    if(originMarker){
        updateLinkOriginMarker();
    }

    return true;
}


void LinkShapeOverwriteItem::Impl::extractShapeNodes(SgNodePath& nodePath)
{
    shapeNode = static_cast<SgShape*>(nodePath.back());

    int directPathTopIndex = nodePath.size() - 1;
    offsetTransform.reset();
    for(int i = nodePath.size() - 2; i >= 0; --i){
        auto group = nodePath[i]->toGroupNode();
        if(group->numChildren() >= 2){
            break;
        }
        directPathTopIndex = i;
        offsetTransform = dynamic_cast<SgPosTransform*>(group);
    }
    if(!offsetTransform){
        offsetTransform = new SgPosTransform;
        SgNode* child = nodePath[directPathTopIndex];
        if(directPathTopIndex > 0){
            auto parent = nodePath[directPathTopIndex - 1]->toGroupNode();
            parent->moveChildrenTo(offsetTransform);
            parent->addChild(offsetTransform);
        } else {
            offsetTransform->addChild(child);
        }
    }
}


Link* LinkShapeOverwriteItem::link()
{
    return impl->link;
}


SgShape* LinkShapeOverwriteItem::shapeNode()
{
    return impl->shapeNode;
}


void LinkShapeOverwriteItem::setShapeNode(SgShape* shapeNode)
{
    if(impl->shapeNode){
        impl->offsetTransform->removeChild(impl->shapeNode);
    }
    impl->shapeNode = shapeNode;
    impl->offsetTransform->addChild(shapeNode);
}


Isometry3 LinkShapeOverwriteItem::shapeOffset() const
{
    return impl->offsetTransform->T();
}


void LinkShapeOverwriteItem::setShapeOffset(const Isometry3& T)
{
    impl->offsetTransform->setPosition(T);
    impl->offsetTransform->notifyUpdate();
    impl->sigOffsetChanged();
    notifyUpdate();
}


void LinkShapeOverwriteItem::onDisconnectedFromBodyItem()
{
    bodyOverwrite()->unregisterLinkShapeOverwriteItem(this);
    impl->clear();
}


void LinkShapeOverwriteItem::cancelOverwriting()
{
    if(impl->orgLinkShape){
        auto& link = impl->link;
        link->clearShapeNodes();
        for(auto& node : *impl->orgLinkShape){
            link->addShapeNode(node);
        }
        link->shape()->notifyUpdate();
    }

    bodyOverwrite()->unregisterLinkShapeOverwriteItem(this);
    impl->clear();
    removeFromParentItem();
}


LocationProxyPtr LinkShapeOverwriteItem::getLocationProxy()
{
    if(!impl->linkShapeLocation){
        impl->linkShapeLocation = new LinkShapeLocation(impl);
    }
    return impl->linkShapeLocation;
}


namespace {

LinkShapeLocation::LinkShapeLocation(LinkShapeOverwriteItem::Impl* impl)
    : LocationProxy(OffsetLocation),
      impl(impl)
{
    setEditable(false);
}
    

Isometry3 LinkShapeLocation::getLocation() const
{
    return impl->offsetTransform->T();
}


bool LinkShapeLocation::setLocation(const Isometry3& T)
{
    impl->offsetTransform->setPosition(T);
    impl->offsetTransform->notifyUpdate();
    impl->self->notifyUpdate();
    return true;
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
    return impl->sigOffsetChanged;
}

}


SgNode* LinkShapeOverwriteItem::getScene()
{
    if(!impl->originMarker){
        impl->updateLinkOriginMarker();
    }
    return impl->originMarker;
}


void LinkShapeOverwriteItem::Impl::updateLinkOriginMarker()
{
    if(!originMarker){
        originMarker = new PositionDragger(
            PositionDragger::TranslationAxes, PositionDragger::PositiveOnlyHandle);
        originMarker->setDragEnabled(false);
        originMarker->setOverlayMode(true);
        originMarker->setPixelSize(96, 3);
        originMarker->setDisplayMode(PositionDragger::DisplayInEditMode);
    }
    if(link){
        originMarker->setPosition(link->T());
        originMarker->notifyUpdate();
    }
}


void LinkShapeOverwriteItem::doPutProperties(PutPropertyFunction& putProperty)
{
    Item::doPutProperties(putProperty);
}


bool LinkShapeOverwriteItem::store(Archive& archive)
{
    if(impl->link){
        if(!impl->link->isRoot()){
            archive.write("link", impl->link->name());
        }
        StdSceneWriter sceneWriter;
        sceneWriter.setFilePathVariableProcessor(archive.filePathVariableProcessor());

        string uri, absoluteUri;
        auto mesh = impl->shapeNode->mesh();
        if(mesh){
            if(mesh->uri() == bodyItem()->filePath()){
                uri = mesh->uri();
                absoluteUri = mesh->absoluteUri();
                // The uri of the mesh is temporarily cleared to omit the description of the mesh node
                // so that the redundant mesh loading can be avoided in restoring the overwriting.
                mesh->clearUri();
            }
        }
        
        if(auto sceneArchive = sceneWriter.writeScene(impl->offsetTransform)){
            archive.insert("overwrite_shape", sceneArchive);
        }
        
        if(!uri.empty()){
            mesh->setUri(uri, absoluteUri);
        }
    }
    
    return true;
}


bool LinkShapeOverwriteItem::restore(const Archive& archive)
{
    auto bodyItem = archive.currentParentItem()->findOwnerItem<BodyItem>(true);
    if(!bodyItem){
        MessageView::instance()->putln(
            format(_("The target body item of \"{0}\" is not found."), displayName()),
            MessageView::Error);
        return false;
    }
    auto body = bodyItem->body();
    auto link = body->rootLink();
    string linkName;
    if(archive.read("link", linkName)){
        link = body->link(linkName);
    }
    if(!link){
        MessageView::instance()->putln(
            format(_("The target link \"{0}\" of \"{1}\" is not found."), linkName, displayName()),
            MessageView::Error);
        return false;
    }

    SgNodePtr restoredShape;
    auto node = archive.find("overwrite_shape");
    if(node->isValid()){
        StdSceneReader sceneReader;
        sceneReader.setFilePathVariableProcessor(archive.filePathVariableProcessor());
        sceneReader.setAngleUnit(StdSceneReader::DEGREE);
        restoredShape = sceneReader.readNode(node->toMapping());
    }

    return impl->overwriteLinkShape(bodyItem, link, restoredShape, nullptr);
}
