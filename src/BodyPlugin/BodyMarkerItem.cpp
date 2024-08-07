#include "BodyMarkerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/BodyItem>
#include <cnoid/SceneGraph>
#include <cnoid/SceneMarkers>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <cnoid/Format>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

int getSceneMarkerType(int type)
{
    switch(type){
    case BodyMarkerItem::CROSS_MARKER:
        return SceneMarker::CROSS_MARKER;
    case BodyMarkerItem::SPHERE_MARKER:
        return SceneMarker::SPHERE_MARKER;
    case BodyMarkerItem::AXES_MARKER:
        return SceneMarker::AXES_MARKER;
    default:
        break;
    }
    return SceneMarker::NO_MARKER;
}

}

namespace cnoid {

class BodyMarkerItem::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    BodyMarkerItem* self;
    BodyItem* bodyItem;
    Link* targetLink;
    Isometry3 T_node;
    Isometry3 localPosition;
    SceneMarkerPtr marker;
    SgSwitchableGroupPtr switchableMarker;
    SgUpdate markerUpdate;
    ScopedConnection connection;
    Selection markerType;
    string targetLinkName;
    string targetNodeName;

    Impl(BodyMarkerItem* self);
    Impl(BodyMarkerItem* self, const Impl& org);
    void setBodyItem(BodyItem* bodyItem);
    bool updateTarget();
    bool findNode();
    bool findNode(SgNode* node, Affine3 T);
    void updateMarkerPosition();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


void BodyMarkerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<BodyMarkerItem>(N_("BodyMarkerItem"));
    im.addCreationPanel<BodyMarkerItem>();
}


BodyMarkerItem::BodyMarkerItem()
{
    impl = new Impl(this);
}


BodyMarkerItem::BodyMarkerItem(const BodyMarkerItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
    setName(org.name());
}


BodyMarkerItem::Impl::Impl(BodyMarkerItem* self)
    : self(self),
      markerType(BodyMarkerItem::N_MARKER_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    bodyItem = nullptr;
    targetLink = nullptr;
    localPosition.setIdentity();

    markerType.setSymbol(BodyMarkerItem::CROSS_MARKER, N_("Cross"));
    markerType.setSymbol(BodyMarkerItem::SPHERE_MARKER, N_("Sphere"));
    markerType.setSymbol(BodyMarkerItem::AXES_MARKER, N_("Axes"));
    markerType.select(BodyMarkerItem::CROSS_MARKER);

    marker = new SceneMarker;
    switchableMarker = new SgSwitchableGroup;
    switchableMarker->setTurnedOn(false);
    switchableMarker->addChild(marker);
}


BodyMarkerItem::Impl::Impl(BodyMarkerItem* self, const Impl& org)
    : Impl(self)
{
    targetLinkName = org.targetLinkName;
    targetNodeName = org.targetNodeName;
    localPosition = org.localPosition;
    markerType = org.markerType;
}


BodyMarkerItem::~BodyMarkerItem()
{
    delete impl;
}


bool BodyMarkerItem::setName(const std::string& name)
{
    impl->switchableMarker->setName(name);
    Item::setName(name);
    return true;
}


Item* BodyMarkerItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new BodyMarkerItem(*this);
}


void BodyMarkerItem::onTreePathChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>());
}


void BodyMarkerItem::Impl::setBodyItem(BodyItem* bodyItem)
{
    if(bodyItem != this->bodyItem){
        this->bodyItem = bodyItem;
        updateTarget();
    }
}


SgNode* BodyMarkerItem::getScene()
{
    if(impl->marker->empty()){
        impl->marker->updateMarker();
    }
    return impl->switchableMarker;
}


int BodyMarkerItem::markerType() const
{
    return impl->markerType.which();
}


void BodyMarkerItem::setMarkerType(int type)
{
    auto& marker = impl->marker;
    if(type != impl->markerType.which()){
        impl->markerType.select(type);
        marker->setMarkerType(getSceneMarkerType(type));
        if(!marker->empty()){
            marker->updateMarker(true);
        }
    }
}


double BodyMarkerItem::markerSize() const
{
    return impl->marker->markerSize();
}


void BodyMarkerItem::setMarkerSize(double size)
{
    auto& marker = impl->marker;
    if(size != marker->markerSize()){
        marker->setMarkerSize(size);
        if(!marker->empty()){
            marker->updateMarker(true);
        }
    }
}


const Vector3f& BodyMarkerItem::markerColor() const
{
    return impl->marker->color();
}


void BodyMarkerItem::setMarkerColor(const Vector3f& color)
{
    auto& marker = impl->marker;
    if(color != marker->color()){
        marker->setColor(color);
        marker->notifyUpdate();
    }
}
        

bool BodyMarkerItem::setTargetLink(const std::string& name)
{
    impl->targetLinkName = name;
    return impl->updateTarget();
}


bool BodyMarkerItem::setTargetNode(const std::string& name)
{
    impl->targetNodeName = name;
    return impl->updateTarget();
}


bool BodyMarkerItem::Impl::updateTarget()
{
    targetLink = nullptr;
    T_node.setIdentity();
    connection.disconnect();
    bool isValid = false;

    if(bodyItem){
        isValid = true; 
        auto mv = MessageView::instance();

        if(!targetLinkName.empty()){
            targetLink = bodyItem->body()->link(targetLinkName);
            if(!targetLink){
                isValid = false;
                mv->putln(
                    formatR(_("Target link \"{0}\" of \"{1}\" is not found."), targetLinkName, self->displayName()),
                    MessageView::Warning);
            }
        }
        if(isValid && !targetNodeName.empty()){
            if(!findNode()){
                targetLink = nullptr;
                isValid = false;
                mv->putln(
                    formatR(_("Target node \"{0}\" of \"{1}\" is not found."), targetNodeName, self->displayName()),
                    MessageView::Warning);
            }
        }
        if(isValid){
            connection.reset(
                bodyItem->sigKinematicStateChanged().connect(
                    [&](){ updateMarkerPosition(); } ));
            SgUpdate update;
            switchableMarker->setTurnedOn(true, update);
            updateMarkerPosition();
        }
    }

    if(!isValid){
        switchableMarker->setTurnedOn(false);
    }

    return isValid;
}


bool BodyMarkerItem::Impl::findNode()
{
    bool found = false;
    if(targetLink){
        found = findNode(targetLink->shape(), Affine3::Identity());
    } else {
        for(auto& link : bodyItem->body()->links()){
            if(findNode(link->shape(), Affine3::Identity())){
                targetLink = link;
                found = true;
                break;
            }
        }
    }
    return found;
}


/**
   \todo Use SgNode::findNode instead of this implementation
*/
bool BodyMarkerItem::Impl::findNode(SgNode* node, Affine3 T)
{
    if(node->name() == targetNodeName){
        T_node = convertToIsometryWithOrthonormalization(T);
        return true;
    }
    if(auto group = dynamic_cast<SgGroup*>(node)){
        if(auto transform = dynamic_cast<SgTransform*>(node)){
            Affine3 T0;
            transform->getTransform(T0);
            T = T * T0;
        }
        for(auto& child : *group){
            if(findNode(child, T)){
                return true;
            }
        }
    }
    return false;
}


void BodyMarkerItem::setOffsetPosition(const Isometry3& T)
{
    impl->localPosition = T;
    impl->updateMarkerPosition();
}
    

void BodyMarkerItem::Impl::updateMarkerPosition()
{
    if(targetLink && marker){
        marker->setPosition(targetLink->T() * T_node * localPosition);
        marker->notifyUpdate(markerUpdate);
    }
}

    
void BodyMarkerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void BodyMarkerItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Marker type"), markerType,
                [&](int type){ self->setMarkerType(type); return true; });

    putProperty(_("Link"), targetLinkName,
                [&](const string& name){ return self->setTargetLink(name); });

    putProperty(_("Node"), targetNodeName,
                [&](const string& name){ return self->setTargetNode(name); });

    putProperty(_("Translation"), str(Vector3(localPosition.translation())),
                [&](const string& value){
                    Vector3 p;
                    if(toVector3(value, p)){
                        localPosition.translation() = p;
                        updateMarkerPosition();
                        return true;
                    }
                    return false;
                });

    Vector3 rpy(TO_DEGREE * rpyFromRot(localPosition.linear()));
    putProperty("RPY", str(rpy), [&](const string& value){
            Vector3 rpy;
            if(toVector3(value, rpy)){
                localPosition.linear() = rotFromRpy(TO_RADIAN * rpy);
                updateMarkerPosition();
                return true;
            }
            return false;
        });

    putProperty(_("Size"), self->markerSize(),
                [&](double size){ self->setMarkerSize(size); return true; });

    putProperty(_("Color"), str(self->markerColor()),
                [&](const string& value){
                    Vector3f color;
                    if(toVector3(value, color)){
                        self->setMarkerColor(color);
                        return true;
                    }
                    return false;
                });
}


bool BodyMarkerItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool BodyMarkerItem::Impl::store(Archive& archive)
{
    archive.write("markerType", markerType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("link", targetLinkName, DOUBLE_QUOTED);
    archive.write("node", targetNodeName, DOUBLE_QUOTED);
    write(archive, "translation", Vector3(localPosition.translation()));
    writeDegreeAngleAxis(archive, "rotation", AngleAxis(localPosition.linear()));
    archive.write("size", self->markerSize());
    write(archive, "color", self->markerColor());
    return true;
}


bool BodyMarkerItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool BodyMarkerItem::Impl::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("markerType", symbol)){
        self->setMarkerType(markerType.index(symbol));
    }

    archive.read("link", targetLinkName);
    archive.read("node", targetNodeName);

    Vector3 translation;
    if(read(archive, "translation", translation)){
        localPosition.translation() = translation;
    }
    AngleAxis aa;
    if(readDegreeAngleAxis(archive, "rotation", aa)){
        localPosition.linear() = aa.toRotationMatrix();
    }

    double size;
    if(archive.read("size", size)){
        self->setMarkerSize(size);
    }

    Vector3f color;
    if(read(archive, "color", color)){
        self->setMarkerColor(color);
    }

    marker->updateMarker(true);
    
    return true;
}
