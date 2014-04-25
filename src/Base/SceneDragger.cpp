/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneDragger.h"
#include <cnoid/SceneUtil>
#include <cnoid/MeshGenerator>
#include <cnoid/MeshNormalGenerator>
#include <boost/bind.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
const double PI = 3.14159265358979323846;
const char* axisNames[3] = { "x", "y", "z" };
}


TranslationDragger::TranslationDragger()
{
    MeshGenerator meshGenerator;

    scale = new SgScaleTransform;
    axisCylinderNormalizedRadius = 0.04;

    for(int i=0; i < 6; ++i){

        const int axis = i / 2;

        SgMaterial* material = new SgMaterial;
        Vector3f color(0.2f, 0.2f, 0.2f);
        color[axis] = 1.0f;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(color);
        material->setAmbientIntensity(0.0f);
        material->setTransparency(0.6f);
        
        SgInvariantGroupPtr group = new SgInvariantGroup;

        double cylinderLength = 1.0;

        if(i % 2 == 0){
            SgShape* cone = new SgShape;
            cone->setMesh(meshGenerator.generateCone(0.1, 0.2, true, true));
            cone->setMaterial(material);
            SgPosTransform* conePos = new SgPosTransform;
            conePos->setTranslation(Vector3(0.0, 0.9, 0.0));
            conePos->addChild(cone);
            group->addChild(conePos);
            cylinderLength = 0.8;
        }

        SgShape* cylinder = new SgShape;
        cylinder->setMesh(
            meshGenerator.generateCylinder(
                axisCylinderNormalizedRadius, cylinderLength, true, true, false));
        cylinder->setMaterial(material);
        SgPosTransform* cylinderPos = new SgPosTransform;
        cylinderPos->setTranslation(Vector3(0.0, cylinderLength / 2.0, 0.0));
        cylinderPos->addChild(cylinder);
        group->addChild(cylinderPos);

        SgPosTransform* arrow = new SgPosTransform;
        arrow->addChild(group);
        arrow->setName(axisNames[axis]);
        scale->addChild(arrow);
        arrows[i] = arrow;
    }

    arrows[0]->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
    arrows[1]->setRotation(AngleAxis( PI / 2.0, Vector3::UnitZ()));
    arrows[3]->setRotation(AngleAxis( PI,       Vector3::UnitX()));
    arrows[4]->setRotation(AngleAxis( PI / 2.0, Vector3::UnitX()));
    arrows[5]->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitX()));

    addChild(scale);

    isContainerMode_ = false;
}


TranslationDragger::TranslationDragger(const TranslationDragger& org)
{
    scale = new SgScaleTransform;
    scale->setScale(org.scale->scale());
    for(int i=0; i < 6; ++i){
        arrows[i] = org.arrows[i];
        scale->addChild(arrows[i]);
    }
    addChild(scale);

    axisCylinderNormalizedRadius = org.axisCylinderNormalizedRadius;
    isContainerMode_ = org.isContainerMode_;
}


TranslationDragger::TranslationDragger(const TranslationDragger& org, SgCloneMap& cloneMap)
    : SgPosTransform(org, cloneMap)
{
    scale = findNodeOfType<SgScaleTransform>();
    for(int i=0; i < 6; ++i){
        arrows[i] = dynamic_cast<SgPosTransform*>(scale->child(i));
    }
    axisCylinderNormalizedRadius = org.axisCylinderNormalizedRadius;
    isContainerMode_ = org.isContainerMode_;
}


SgObject* TranslationDragger::clone(SgCloneMap& cloneMap) const
{
    return new TranslationDragger(*this, cloneMap);
}


double TranslationDragger::radius() const
{
    return scale->scale().x();
}


void TranslationDragger::setRadius(double r)
{
    scale->setScale(r);
}


void TranslationDragger::setContainerMode(bool on)
{
    isContainerMode_ = on;
}


bool TranslationDragger::isContainerMode() const
{
    return isContainerMode_;
}


bool TranslationDragger::isDragging() const
{
    return dragProjector.isDragging();
}


const Vector3& TranslationDragger::draggedTranslation() const
{
    return dragProjector.translation();
}


const Affine3& TranslationDragger::draggedPosition() const
{
    return dragProjector.position();
}


bool TranslationDragger::onButtonPressEvent(const SceneWidgetEvent& event)
{
    int indexOfTopNode = -1;
    int axisIndex = -1;
    const SgNodePath& path = event.nodePath();
    for(size_t i=0; i < path.size(); ++i){
        const SgNode* node = path[i];
        if(node == this){
            indexOfTopNode = i;
        }
        if(indexOfTopNode >= 0){
            const string& name = node->name();
            if(!name.empty()){
                for(int j=0; j < 3; ++j){
                    if(name == axisNames[j]){
                        axisIndex = j;
                        goto exit;
                    }
                }
            }
        }
    }
exit:
    if(axisIndex >= 0){
        SgNodePath::const_iterator axisIter = path.begin() + indexOfTopNode + 1;
        const Affine3 T_global = calcTotalTransform(path.begin(), axisIter);
        dragProjector.setInitialPosition(T_global);
        const Affine3 T_axis = calcTotalTransform(axisIter, path.end());
        const Vector3 p_local = (T_global * T_axis).inverse() * event.point();
        if(p_local.norm() < 2.0 * axisCylinderNormalizedRadius){
            dragProjector.setTranslationAlongViewPlane();
        } else {
            dragProjector.setTranslationAxis(T_global.linear().col(axisIndex));
        }
        if(dragProjector.startTranslation(event)){
            sigTranslationStarted_();
            return true;
        }
    }
    return false;
}


bool TranslationDragger::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.isDragging()){
        sigTranslationFinished_();
        dragProjector.resetDragMode();
        return true;
    }
    return false;
}


bool TranslationDragger::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.dragTranslation(event)){
        if(isContainerMode_){
            setPosition(dragProjector.position());
            notifyUpdate();
        }
        sigTranslationDragged_();
        return true;
    }
    return false;
}


void TranslationDragger::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.isDragging()){
        sigTranslationFinished_();
        dragProjector.resetDragMode();
    }
}


RotationDragger::RotationDragger()
{
    static const int numDivisions = 30;
    static const float beltWidthRatio = 0.1f;
    static const int axes[3][3] = { { 0, 1, 2 }, { 1, 0, 2 }, { 2, 1, 0 } };
    static const double PI = 3.14159265358979323846;

    MeshNormalGenerator normalGenerator;
    scale = new SgScaleTransform;

    // make dragger belts
    for(int i=0; i < 3; ++i){
        
        SgMaterial* material = new SgMaterial;
        Vector3f color(0.2f, 0.2f, 0.2f);
        color[i] = 1.0f;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(color);
        material->setAmbientIntensity(0.0f);
        material->setTransparency(0.6f);

        const int x = axes[i][0];
        const int y = axes[i][1];
        const int z = axes[i][2];

        SgMeshPtr mesh = new SgMesh;
        SgVertexArray& vertices = *mesh->getOrCreateVertices();
        vertices.reserve(numDivisions * 2);
        mesh->reserveNumTriangles(numDivisions * 2);
        
        for(int j=0; j < numDivisions; ++j){
            const double theta = j * 2.0 * PI / numDivisions;
            Vector3f v1, v2;
            v1[x] =  beltWidthRatio;
            v2[x] = -beltWidthRatio;
            v1[y] = v2[y] = cos(theta);
            v1[z] = v2[z] = sin(theta);
            vertices.push_back(v1);
            vertices.push_back(v2);

            const int s = j * 2;
            const int t = ((j + 1) % numDivisions) * 2;
            mesh->addTriangle(s, t, t+1);
            mesh->addTriangle(s, t+1, s+1);
        }
        normalGenerator.generateNormals(mesh);

        SgShape* belt = new SgShape;
        belt->setMesh(mesh);
        belt->setMaterial(material);
        belt->setName(axisNames[i]);
        scale->addChild(belt);
        belts[i] = belt;
    }

    addChild(scale);

    isContainerMode_ = false;
}


RotationDragger::RotationDragger(const RotationDragger& org)
{
    scale = new SgScaleTransform;
    scale->setScale(org.scale->scale());
    const int n = org.scale->numChildren();
    for(int i=0; i < 3; ++i){
        belts[i] = org.belts[i];
        scale->addChild(belts[i]);
    }
    addChild(scale);
    
    isContainerMode_ = org.isContainerMode_;
}


RotationDragger::RotationDragger(const RotationDragger& org, SgCloneMap& cloneMap)
    : SgPosTransform(org, cloneMap)
{
    scale = findNodeOfType<SgScaleTransform>();
    for(int i=0; i < 3; ++i){
        belts[i] = dynamic_cast<SgShape*>(scale->child(i));
    }
    isContainerMode_ = org.isContainerMode_;
}


SgObject* RotationDragger::clone(SgCloneMap& cloneMap) const
{
    return new RotationDragger(*this, cloneMap);
}


void RotationDragger::setRadius(double r)
{
    scale->setScale(r);
}


void RotationDragger::setContainerMode(bool on)
{
    isContainerMode_ = on;
}


bool RotationDragger::isContainerMode() const
{
    return isContainerMode_;
}


bool RotationDragger::isDragging() const
{
    return dragProjector.isDragging();
}


const AngleAxis& RotationDragger::draggedAngleAxis() const
{
    return dragProjector.rotationAngleAxis();
}


const Affine3& RotationDragger::draggedPosition() const
{
    return dragProjector.position();
}


bool RotationDragger::onButtonPressEvent(const SceneWidgetEvent& event)
{
    SgShape* belt = dynamic_cast<SgShape*>(event.nodePath().back());
    int axisIndex = -1;
    if(belt){
        if(belt->name() == "x"){
            axisIndex = 0;
        } else if(belt->name() == "y"){
            axisIndex = 1;
        } else if(belt->name() == "z"){
            axisIndex = 2;
        }
        if(axisIndex >= 0){
            Affine3 T = calcTotalTransform(event.nodePath(), this);
            dragProjector.setInitialPosition(T);
            dragProjector.setRotationAxis(T.linear().col(axisIndex));
            if(dragProjector.startRotation(event)){
                sigRotationStarted_();
                return true;
            }
        }
    }
    return false;
}


bool RotationDragger::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.isDragging()){
        sigRotationFinished_();
        dragProjector.resetDragMode();
        return true;
    }
    return false;
}


bool RotationDragger::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.dragRotation(event)){
        if(isContainerMode_){
            setPosition(dragProjector.position());
            notifyUpdate();
        }
        sigRotationDragged_(dragProjector.rotationAngleAxis());
        return true;
    }
    return false;
}


void RotationDragger::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.isDragging()){
        sigRotationFinished_();
        dragProjector.resetDragMode();
    }
}


PositionDragger::PositionDragger()
{
    translationDragger_ = new TranslationDragger;
    rotationDragger_ = new RotationDragger;
    initalizeDraggers();
    isDraggerAlwaysShown_ = false;
    isContainerMode_ = false;
}


PositionDragger::PositionDragger(const PositionDragger& org)
{
    translationDragger_ = new TranslationDragger(*org.translationDragger_);
    rotationDragger_ = new RotationDragger(*org.rotationDragger_);
    initalizeDraggers();
    isContainerMode_ = org.isContainerMode_;
    isDraggerAlwaysShown_ = org.isDraggerAlwaysShown_;
}


PositionDragger::PositionDragger(const PositionDragger& org, SgCloneMap& cloneMap)
{
    translationDragger_ = new TranslationDragger(*org.translationDragger_, cloneMap);
    rotationDragger_ = new RotationDragger(*org.rotationDragger_, cloneMap);
    initalizeDraggers();
    isContainerMode_ = org.isContainerMode_;
    isDraggerAlwaysShown_ = org.isDraggerAlwaysShown_;
}


void PositionDragger::initalizeDraggers()
{
    translationDragger_->sigTranslationStarted().connect(ref(sigDragStarted_));
    translationDragger_->sigTranslationDragged().connect(
        bind(&PositionDragger::onPositionDragged, this));
    translationDragger_->sigTranslationFinished().connect(ref(sigDragFinished_));
    
    rotationDragger_->sigRotationStarted().connect(ref(sigDragStarted_));
    rotationDragger_->sigRotationDragged().connect(
        bind(&PositionDragger::onPositionDragged, this));
    rotationDragger_->sigRotationFinished().connect(ref(sigDragFinished_));
}


SgObject* PositionDragger::clone(SgCloneMap& cloneMap) const
{
    return new PositionDragger(*this, cloneMap);
}


void PositionDragger::setRadius(double r, double translationAxisRatio)
{
    translationDragger_->setRadius(r * translationAxisRatio);
    rotationDragger_->setRadius(r);
}


void PositionDragger::setContainerMode(bool on)
{
    isContainerMode_ = on;
}


bool PositionDragger::isContainerMode() const
{
    return isContainerMode_;
}


void PositionDragger::setDraggerAlwaysShown(bool on)
{
    if(on != isDraggerAlwaysShown_){
        if(on){
            addChild(translationDragger_);
            addChild(rotationDragger_, true);
        } else {
            removeChild(translationDragger_);
            removeChild(rotationDragger_, true);
        }
    }
    isDraggerAlwaysShown_ = on;
}


bool PositionDragger::isDraggerAlwaysShown() const
{
    return isDraggerAlwaysShown_;
}


Affine3 PositionDragger::draggedPosition() const
{
    if(rotationDragger_->isDragging()){
        return rotationDragger_->draggedPosition();
    } else if(translationDragger_->isDragging()){
        return translationDragger_->draggedPosition();
    } else if(dragProjector.isDragging()){
        return dragProjector.position();
    } else {
        return T();
    }
}


void PositionDragger::onPositionDragged()
{
    if(isContainerMode_){
        setPosition(draggedPosition());
        notifyUpdate();
    }
    sigPositionDragged_();
}


bool PositionDragger::onButtonPressEvent(const SceneWidgetEvent& event)
{
    if(!isDraggerAlwaysShown_){
        bool added = false;
        if(!contains(translationDragger_)){
            addChild(translationDragger_);
            added = true;
        }
        if(!contains(rotationDragger_)){
            addChild(rotationDragger_);
            added = true;
        }
        if(added){
            notifyUpdate(SgUpdate::ADDED);
        }
    }
    dragProjector.setInitialPosition(T());
    dragProjector.setTranslationAlongViewPlane();
    return dragProjector.startTranslation(event);
}


bool PositionDragger::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.isDragging()){
        dragProjector.resetDragMode();
        return true;
    }
    return false;
}


bool PositionDragger::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(dragProjector.drag(event)){
        setPosition(dragProjector.position());
        notifyUpdate();
        return true;
    }
    return false;
}


void PositionDragger::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    dragProjector.resetDragMode();
}


void PositionDragger::onFocusChanged(const SceneWidgetEvent& event, bool on)
{
    if(!isDraggerAlwaysShown_){
        if(on){
            addChild(translationDragger_);
            addChild(rotationDragger_, true);
        } else {
            removeChild(translationDragger_);
            removeChild(rotationDragger_, true);
        }
    }
}
