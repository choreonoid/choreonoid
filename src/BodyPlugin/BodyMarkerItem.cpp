/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyMarkerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/BodyItem>
#include <cnoid/SceneGraph>
#include <cnoid/MeshGenerator>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <boost/format.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;

namespace cnoid {

class BodyMarkerItemImpl
{
public:
    BodyMarkerItem* self;
    BodyItem* bodyItem;
    Link* targetLink;
    Position T_node;
    Position localPosition;
    SgPosTransformPtr marker;
    SgSwitchPtr markerSwitch;
    SgMaterialPtr markerMaterial;
    SgUpdate markerUpdate;
    ScopedConnection connection;
    string targetLinkName;
    string targetNodeName;
    Selection markerType;
    double markerSize;
    Vector3f markerColor;
    MeshGenerator meshGenerator;

    BodyMarkerItemImpl(BodyMarkerItem* self);
    BodyMarkerItemImpl(BodyMarkerItem* self, const BodyMarkerItemImpl& org);
    void updateMarker();
    void setCross();
    void setSphere();
    void setAxisArrows();
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
    impl = new BodyMarkerItemImpl(this);
}


BodyMarkerItem::BodyMarkerItem(const BodyMarkerItem& org)
    : Item(org)
{
    impl = new BodyMarkerItemImpl(this, *org.impl);
    setName(org.name());
}


BodyMarkerItemImpl::BodyMarkerItemImpl(BodyMarkerItem* self)
    : self(self),
      markerType(BodyMarkerItem::N_MARKER_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    bodyItem = nullptr;
    targetLink = nullptr;
    localPosition.setIdentity();

    markerType.setSymbol(BodyMarkerItem::CROSS_MARKER, N_("Cross"));
    markerType.setSymbol(BodyMarkerItem::SPHERE_MARKER, N_("Sphere"));
    markerType.setSymbol(BodyMarkerItem::AXIS_ARROWS_MARKER, N_("Axis arrows"));
    markerType.select(BodyMarkerItem::CROSS_MARKER);

    markerSize = 0.1;
    markerColor << 1.0f, 1.0f, 0.0f;

    marker = new SgPosTransform;
    markerSwitch = new SgSwitch;
    markerSwitch->turnOff();
    marker->addChild(markerSwitch);
    markerMaterial = new SgMaterial;
    markerMaterial->setDiffuseColor(Vector3f::Zero());
    markerMaterial->setEmissiveColor(markerColor);
    markerMaterial->setAmbientIntensity(0.0f);
}


BodyMarkerItemImpl::BodyMarkerItemImpl(BodyMarkerItem* self, const BodyMarkerItemImpl& org)
    : BodyMarkerItemImpl(self)
{
    targetLinkName = org.targetLinkName;
    targetNodeName = org.targetNodeName;
    localPosition = org.localPosition;
    markerType = org.markerType;
    markerSize = org.markerSize;
    markerColor = org.markerColor;
    markerMaterial->setEmissiveColor(markerColor);
}


BodyMarkerItem::~BodyMarkerItem()
{
    delete impl;
}


void BodyMarkerItem::setName(const std::string& name)
{
    Item::setName(name);
}


Item* BodyMarkerItem::doDuplicate() const
{
    return new BodyMarkerItem(*this);
}


void BodyMarkerItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>());
}


void BodyMarkerItemImpl::setBodyItem(BodyItem* bodyItem)
{
    if(bodyItem != this->bodyItem){
        this->bodyItem = bodyItem;
        updateTarget();
    }
}


SgNode* BodyMarkerItem::getScene()
{
    if(impl->markerSwitch->empty()){
        impl->updateMarker();
    }
    return impl->marker;
}


void BodyMarkerItem::setMarkerType(int type)
{
    if(type != impl->markerType.which() || impl->marker->empty()){
        impl->markerType.select(type);
        impl->updateMarker();
    }
}


void BodyMarkerItem::setMarkerSize(double size)
{
    if(size != impl->markerSize){
        impl->markerSize = size;
        if(!impl->markerSwitch->empty()){
            impl->updateMarker();
        }
    }
}


void BodyMarkerItem::setMarkerColor(const Vector3f& color)
{
    if(color != impl->markerColor){
        impl->markerColor = color;
        impl->markerMaterial->setEmissiveColor(color);
        impl->markerMaterial->notifyUpdate();
    }
}
        

void BodyMarkerItemImpl::updateMarker()
{
    switch(markerType.which()){
    case BodyMarkerItem::CROSS_MARKER:
        setCross();
        break;
    case BodyMarkerItem::SPHERE_MARKER:
        setSphere();
        break;
    case BodyMarkerItem::AXIS_ARROWS_MARKER:
        setAxisArrows();
        break;
    defautl:
        break;
    }
}


void BodyMarkerItemImpl::setCross()
{
    markerSwitch->clearChildren();

    const float p = markerSize;
    auto vertices = new SgVertexArray {
        {   -p, 0.0f, 0.0f },
        {    p, 0.0f, 0.0f },
        { 0.0f,   -p, 0.0f },
        { 0.0f,    p, 0.0f },
        { 0.0f, 0.0f,   -p },
        { 0.0f, 0.0f,    p }
    };
    
    SgLineSet* lineSet = new SgLineSet;
    lineSet->setVertices(vertices);
    lineSet->setNumLines(3);
    lineSet->setLine(0, 0, 1);
    lineSet->setLine(1, 2, 3);
    lineSet->setLine(2, 4, 5);

    lineSet->setMaterial(markerMaterial);

    markerSwitch->addChild(lineSet, true);
}


void BodyMarkerItemImpl::setSphere()
{
    markerSwitch->clearChildren();
    SgShape* shape = new SgShape;
    shape->setMesh(meshGenerator.generateSphere(markerSize / 2.0));
    shape->setMaterial(markerMaterial);
    markerSwitch->addChild(shape, true);
}


void BodyMarkerItemImpl::setAxisArrows()
{
    markerSwitch->clearChildren();

    double r1 = markerSize * 0.1;
    double h1 = markerSize * 0.7;
    double r2 = markerSize * 0.2;
    double h2 = markerSize * 0.3;
    
    SgMeshPtr mesh = meshGenerator.generateArrow(r1, h1, r2, h2);
    mesh->translate(Vector3f(0.0f, h1 / 2.0, 0.0f));

    for(int i=0; i < 3; ++i){
        SgShape* shape = new SgShape;
        shape->setMesh(mesh);

        SgMaterial* material = new SgMaterial;
        Vector3f color(0.2f, 0.2f, 0.2f);
        color[i] = 1.0f;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(color);
        material->setAmbientIntensity(0.0f);
        shape->setMaterial(material);
            
        SgPosTransform* arrow = new SgPosTransform;
        arrow->addChild(shape);
        if(i == 0){
            arrow->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
        } else if(i == 2){
            arrow->setRotation(AngleAxis( PI / 2.0, Vector3::UnitX()));
        }

        markerSwitch->addChild(arrow, true);
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


bool BodyMarkerItemImpl::updateTarget()
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
                mv->putln(MessageView::WARNING,
                          format(_("Target link \"%1%\" of \"%2%\" is not found."))
                          % targetLinkName % self->name());
            }
        }
        if(isValid && !targetNodeName.empty()){
            if(!findNode()){
                targetLink = nullptr;
                isValid = false;
                mv->putln(MessageView::WARNING,
                          format(_("Target node \"%1%\" of \"%2%\" is not found."))
                          % targetNodeName % self->name());
            }
        }
        if(isValid){
            connection.reset(
                bodyItem->sigKinematicStateChanged().connect(
                    [&](){ updateMarkerPosition(); } ));
            markerSwitch->turnOn();
            updateMarkerPosition();
        }
    }

    if(!isValid){
        markerSwitch->turnOff(true);
    }

    return isValid;
}


bool BodyMarkerItemImpl::findNode()
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


bool BodyMarkerItemImpl::findNode(SgNode* node, Affine3 T)
{
    if(node->name() == targetNodeName){
        T_node = T;
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


void BodyMarkerItem::setOffsetPosition(const Position& T)
{
    impl->localPosition = T;
    impl->updateMarkerPosition();
}
    

void BodyMarkerItemImpl::updateMarkerPosition()
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


void BodyMarkerItemImpl::doPutProperties(PutPropertyFunction& putProperty)
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

    putProperty(_("Size"), markerSize,
                [&](double size){ self->setMarkerSize(size); return true; });

    putProperty(_("Color"), str(markerColor),
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


bool BodyMarkerItemImpl::store(Archive& archive)
{
    archive.write("markerType", markerType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("link", targetLinkName, DOUBLE_QUOTED);
    archive.write("node", targetNodeName, DOUBLE_QUOTED);
    write(archive, "translation", Vector3(localPosition.translation()));
    write(archive, "rotation", AngleAxis(localPosition.linear()));
    archive.write("size", markerSize);
    write(archive, "color", markerColor);
    return true;
}


bool BodyMarkerItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool BodyMarkerItemImpl::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("markerType", symbol)){
        markerType.select(symbol);
    }

    archive.read("link", targetLinkName);
    archive.read("node", targetNodeName);

    Vector3 translation;
    if(read(archive, "translation", translation)){
        localPosition.translation() = translation;
    }
    AngleAxis a;
    if(read(archive, "rotation", a)){
        localPosition.linear() = a.toRotationMatrix();
    }

    archive.read("size", markerSize);

    Vector3f color;
    if(read(archive, "color", color)){
        self->setMarkerColor(color);
    }

    updateMarker();
    
    return true;
}
