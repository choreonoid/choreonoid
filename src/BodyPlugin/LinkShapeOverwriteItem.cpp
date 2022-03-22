#include "LinkShapeOverwriteItem.h"
#include "BodyOverwriteAddon.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/Link>
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
    LinkPtr link;
    SgPosTransformPtr offsetTransform;
    Signal<void()> sigOffsetChanged;
    vector<SgNodePath> shapeNodePaths;
    SgGroupPtr orgLinkShape;
    ref_ptr<LinkShapeLocation> linkShapeLocation;
    PositionDraggerPtr originMarker;
    ScopedConnection bodyItemConnection;
    
    Impl(LinkShapeOverwriteItem* self);
    Impl(LinkShapeOverwriteItem* self, const Impl& org);
    void clear();
    bool overwriteLinkShape(BodyItem* bodyItem, Link* link, SgNode* newShape, SgGroup* orgLinkShapeOfDuplicationOrgItem);
    void extractShapeNodeSet(vector<SgNodePath>& nodePaths);
    void extractOrInsertOffsetTransform(SgNode* node);
    void extractOrInsertOffsetTransform(vector<SgNodePath>& nodePaths);
    void setShapeNode(SgShape* shapeNode);
    void updateLinkOriginMarker();
    bool store(Archive& archive);
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


/**
   \note If the return value of BodyOverwriteAddon::checkIfSingleShapeBody is false,
   only the offset position can be modified, and accessing and editing the shape nodes are not available.
   \todo Extract multiple SgShape nodes and make the shapePaths function available.
*/
bool LinkShapeOverwriteItem::Impl::overwriteLinkShape
(BodyItem* bodyItem, Link* link, SgNode* newShape, SgGroup* orgLinkShapeOfDuplicationOrgItem)
{
    if(self->bodyItem()){
        return false; // This item has been overwriting a body item
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

    if(orgLinkShapeOfDuplicationOrgItem){
        orgLinkShape = orgLinkShapeOfDuplicationOrgItem;
    } else {
        if(!orgLinkShape){
            orgLinkShape = new SgGroup;
        } else {
            orgLinkShape->clearChildren();
        }
        link->shape()->copyChildrenTo(orgLinkShape);
    }
    
    offsetTransform.reset();
    shapeNodePaths.clear();

    SceneNodeExtractor nodeExtractor;

    if(newShape){
        shapeNodePaths = nodeExtractor.extractNodes<SgShape>(newShape, true);
        extractOrInsertOffsetTransform(newShape);
    }

    /*
      The following code is temporary and cloning the shape nodes should not be performed.
      Instead of it, the code to overwrite a node must create a new node instance and replace
      the existing node with it.
    */
    CloneMap cloneMap;
    SgNodePtr shapeClone = cloneMap.getClone(link->shape());
    
    vector<SgNodePath> existingShapeNodePaths = nodeExtractor.extractNodes<SgShape>(shapeClone, false);

    if(existingShapeNodePaths.empty()){
        if(!newShape){
            offsetTransform = new SgPosTransform;
        }
    } else {
        if(!newShape){
            extractOrInsertOffsetTransform(existingShapeNodePaths);
            shapeNodePaths = std::move(existingShapeNodePaths);
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

    link->clearShapeNodes();
    link->addShapeNode(offsetTransform, true);

    bodyItemConnection =
        bodyItem->sigKinematicStateChanged().connect(
            [&](){ updateLinkOriginMarker(); });
    
    if(originMarker){
        updateLinkOriginMarker();
    }

    return true;
}


void LinkShapeOverwriteItem::Impl::extractOrInsertOffsetTransform(SgNode* node)
{
    if(auto transformNode = dynamic_cast<SgPosTransform*>(node)){
        offsetTransform = transformNode;
    } else {
        offsetTransform = new SgPosTransform;
        offsetTransform->addChild(node);
    }
}


void LinkShapeOverwriteItem::Impl::extractOrInsertOffsetTransform(vector<SgNodePath>& nodePaths)
{
    SgNode* firstTopNode = nodePaths.front().front();
    offsetTransform = dynamic_cast<SgPosTransform*>(firstTopNode);
    if(offsetTransform){
        for(size_t i=1; i < nodePaths.size(); ++i){
            auto& nodePath = nodePaths[i];
            if(nodePath.front() != firstTopNode){
                offsetTransform.reset();
                break;
            }
        }
    }
    if(!offsetTransform){
        offsetTransform = new SgPosTransform;
        for(auto& nodePath : nodePaths){
            offsetTransform->addChildOnce(nodePath.front());
            nodePath.insert(nodePath.begin(), offsetTransform);
        }
    }
}


Link* LinkShapeOverwriteItem::link()
{
    return impl->link;
}


std::vector<SgNodePath> LinkShapeOverwriteItem::shapePaths()
{
    return impl->shapeNodePaths;
}


SgShape* LinkShapeOverwriteItem::shapeNode()
{
    if(!impl->shapeNodePaths.empty()){
        return static_cast<SgShape*>(impl->shapeNodePaths[0].back().get());
    }
    return nullptr;
}


void LinkShapeOverwriteItem::setShapeNode(SgShape* shapeNode)
{
    impl->setShapeNode(shapeNode);
}


void LinkShapeOverwriteItem::Impl::setShapeNode(SgShape* shapeNode)
{
    if(shapeNodePaths.empty()){
        offsetTransform->addChild(shapeNode);
        SgNodePath path = { offsetTransform, shapeNode };
        shapeNodePaths.emplace_back(std::move(path));
    } else {
        auto& shapeNodePath0 = shapeNodePaths.front();
        auto shapeNode0 = static_cast<SgShape*>(shapeNodePath0.back().get());
        auto parentGroup = dynamic_cast<SgGroup*>(shapeNodePath0.front().get());
        if(parentGroup){
            parentGroup->removeChild(shapeNode0);
            parentGroup->addChild(shapeNode);
            shapeNodePath0.back() = shapeNode;
        }
    }
}


Isometry3 LinkShapeOverwriteItem::shapeOffset() const
{
    if(impl->offsetTransform){
        return impl->offsetTransform->T();
    }
    return Isometry3::Identity();
}


void LinkShapeOverwriteItem::setShapeOffset(const Isometry3& T)
{
    if(impl->offsetTransform){
        impl->offsetTransform->setPosition(T);
        impl->offsetTransform->notifyUpdate();
        impl->sigOffsetChanged();
        notifyUpdate();
    }
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
    return impl->store(archive);
}


bool LinkShapeOverwriteItem::Impl::store(Archive& archive)
{
    if(link && offsetTransform){
        if(!link->isRoot()){
            archive.write("link", link->name());
        }
        StdSceneWriter sceneWriter;
        sceneWriter.setFilePathVariableProcessor(archive.filePathVariableProcessor());
        sceneWriter.setExtModelFileMode(StdSceneWriter::EmbedModels);
        // Temporary configuration. The mesh outout should be enabled if it is explicitly specified.
        sceneWriter.setMeshEnabled(false);

        if(auto sceneArchive = sceneWriter.writeScene(offsetTransform)){
            archive.insert("overwrite_shape", sceneArchive);
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
