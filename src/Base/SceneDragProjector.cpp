/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "SceneDragProjector.h"
#include "SceneProjector.h"
#include <cnoid/SceneCameras>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

enum TranslationMode { TRANSLATION_1D, TRANSLATION_2D, TRANSLATION_VIEW_PLANE };

}

namespace cnoid {

class SceneDragProjectorImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    int dragMode;
    Isometry3 initialPosition;
    Vector3 initialPoint;
    Isometry3 position;
    Vector3 projectedPoint;

    Vector3 rotationAxis;
    Vector3 rotationBaseX;
    Vector3 rotationBaseY;
    AngleAxis rotationAngleAxis;
    Matrix3 rotationMatrix;
    double rotationAngle;
        
    TranslationMode translationMode;
    Vector3 translationAxis;
    Vector3 translationPlaneNormal;
    Vector3 translation;

    std::shared_ptr<SceneProjector> projector;

    SceneDragProjectorImpl();
    bool startRotation(SceneWidgetEvent* event);
    bool dragRotation(SceneWidgetEvent* event);
    bool startTranslation(SceneWidgetEvent* event);
    bool dragTranslation(SceneWidgetEvent* event);
};
}


SceneDragProjector::SceneDragProjector()
{
    impl = new SceneDragProjectorImpl;
}


SceneDragProjectorImpl::SceneDragProjectorImpl()
{
    dragMode = SceneDragProjector::DRAG_NONE;
    initialPosition = Isometry3::Identity();
}


SceneDragProjector::~SceneDragProjector()
{
    delete impl;
}


void SceneDragProjector::resetDragMode()
{
    impl->dragMode = DRAG_NONE;
}


bool SceneDragProjector::isDragging() const
{
    return (impl->dragMode != DRAG_NONE);
}


void SceneDragProjector::setInitialPosition(const Isometry3& T)
{
    impl->initialPosition = T;
    impl->position = T;
}


void SceneDragProjector::setInitialTranslation(const Vector3& p)
{
    impl->initialPosition.translation() = p;
}


void SceneDragProjector::setInitialRotation(const Matrix3& R)
{
    impl->initialPosition.linear() = R;
}


const Isometry3& SceneDragProjector::initialPosition() const
{
    return impl->initialPosition;
}


void SceneDragProjector::setRotationAxis(const Vector3& axis)
{
    impl->rotationAxis = axis.normalized();
}


const Vector3& SceneDragProjector::rotationAxis() const
{
    return impl->rotationAxis;
}


bool SceneDragProjector::startRotation(SceneWidgetEvent* event)
{
    return impl->startRotation(event);
}


bool SceneDragProjectorImpl::startRotation(SceneWidgetEvent* event)
{
    initialPoint = event->point();
    const Vector3 p = initialPosition.translation();
    const Vector3 arm = initialPoint - (rotationAxis.dot(initialPoint - p) * rotationAxis + p);

    const double armLength = arm.norm();
    if(armLength < 1.0e-6){
        dragMode = SceneDragProjector::DRAG_NONE;
        return false;
    }
    
    bool initialized = false;
    
    rotationBaseX = arm.normalized();
    rotationBaseY = rotationAxis.cross(rotationBaseX);

    Vector3 direction;
    if(auto persCamera = dynamic_cast<const SgPerspectiveCamera*>(event->camera())){
        direction = (event->point() - event->cameraPosition().translation()).normalized();
    } else {
        direction = SgCamera::direction(event->cameraPosition());
    }

    if(fabs(rotationAxis.dot(direction)) > 0.13){
        projector = make_shared_aligned<ScenePlaneProjector>(rotationAxis, initialPoint);
        initialized = true;
    } else {
        Quaternion rotation;
        rotation.setFromTwoVectors(Vector3::UnitZ(), rotationAxis);
        auto cprojector =
            make_shared_aligned<SceneCylinderProjector>(
                p, armLength, numeric_limits<double>::max(), rotation);
        if(cprojector->initializeProjection(event)){
            projector = cprojector;
            initialized = true;
        }
    }

    if(initialized){
        dragMode = SceneDragProjector::DRAG_ROTATION;
    }

    return initialized;
}


bool SceneDragProjector::dragRotation(SceneWidgetEvent* event)
{
    return impl->dragRotation(event);
}


bool SceneDragProjectorImpl::dragRotation(SceneWidgetEvent* event)
{
    if(dragMode == SceneDragProjector::DRAG_ROTATION){
        if(projector->project(event, projectedPoint)){
            const Vector3 r = projectedPoint - initialPosition.translation();
            rotationAngle = atan2(r.dot(rotationBaseY), r.dot(rotationBaseX));
            rotationAngleAxis = AngleAxis(rotationAngle, rotationAxis);
            rotationMatrix = rotationAngleAxis;
            position.linear() = rotationMatrix * initialPosition.linear();
            normalizeRotation(position);
            return true;
        }
    }
    return false;
}


int SceneDragProjector::dragMode() const
{
    return impl->dragMode;
}


bool SceneDragProjector::drag(SceneWidgetEvent* event)
{
    switch(impl->dragMode){
    case DRAG_ROTATION: return impl->dragRotation(event);
    case DRAG_TRANSLATION: return impl->dragTranslation(event);
    default: return false;
    }
}


double SceneDragProjector::rotationAngle() const
{
    return impl->rotationAngle;
}


const AngleAxis& SceneDragProjector::rotationAngleAxis() const
{
    return impl->rotationAngleAxis;
}


const Matrix3& SceneDragProjector::rotationMatrix() const
{
    return impl->rotationMatrix;
}


const Vector3& SceneDragProjector::projectedPoint() const
{
    return impl->projectedPoint;
}


const Isometry3& SceneDragProjector::position() const
{
    return impl->position;
}


void SceneDragProjector::setTranslationAxis(const Vector3& axis)
{
    impl->translationAxis = axis;
    impl->translationMode = TRANSLATION_1D;
}


const Vector3& SceneDragProjector::translationAxis() const
{
    return impl->translationAxis;
}


void SceneDragProjector::setTranslationPlaneNormal(const Vector3& normal)
{
    impl->translationPlaneNormal = normal;
    impl->translationMode = TRANSLATION_2D;
}


void SceneDragProjector::setTranslationAlongViewPlane()
{
    impl->translationMode = TRANSLATION_VIEW_PLANE;
}


bool SceneDragProjector::startTranslation(SceneWidgetEvent* event)
{
    return impl->startTranslation(event);
}


bool SceneDragProjectorImpl::startTranslation(SceneWidgetEvent* event)
{
    initialPoint = event->point();
    
    switch(translationMode){

    case TRANSLATION_1D:
    {
        const Isometry3& C = event->cameraPosition();
        const Vector3 eye = C.translation();
        const Vector3 center = SgCamera::direction(C) + eye;
        const Vector3 z = (eye - center).normalized();
        const Vector3& a = translationAxis;
        if(fabs(a.dot(z)) > 0.99){
            return false;
        }
        const Vector3 normal = (AngleAxis(M_PI / 2.0, z) * a).cross(a).normalized();
        projector.reset(new ScenePlaneProjector(normal, initialPoint));
        break;
    }

    case TRANSLATION_2D:
    {
        projector.reset(new ScenePlaneProjector(translationPlaneNormal, initialPoint));
        break;
    }

    case TRANSLATION_VIEW_PLANE:
    {
        const Isometry3& C = event->cameraPosition();
        projector.reset(new ScenePlaneProjector(-SgCamera::direction(C), initialPoint));
        break;
    }
    default:
        dragMode = SceneDragProjector::DRAG_NONE;
        return false;
    }

    dragMode = SceneDragProjector::DRAG_TRANSLATION;
    return true;
}


bool SceneDragProjector::dragTranslation(SceneWidgetEvent* event)
{
    return impl->dragTranslation(event);
}


bool SceneDragProjectorImpl::dragTranslation(SceneWidgetEvent* event)
{
    if(dragMode == SceneDragProjector::DRAG_TRANSLATION){
        if(projector->project(event, projectedPoint)){
            translation = projectedPoint - initialPoint;
            if(translationMode == TRANSLATION_1D){
                translation = translationAxis.dot(translation) * translationAxis;
            }
            position.translation() = initialPosition.translation() + translation;
            return true;
        }
    }
    return false;
}


const Vector3& SceneDragProjector::translation() const
{
    return impl->translation;
}
