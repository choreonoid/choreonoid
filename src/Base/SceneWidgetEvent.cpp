/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidgetEvent.h"
#include "SceneWidget.h"
#include <cnoid/SceneRenderer>
#include <cnoid/SceneCameras>

using namespace cnoid;


SceneWidgetEvent::SceneWidgetEvent()
{
    point_.setZero();
    x_ = 0.0;
    y_ = 0.0;
    pixelSizeRatio_ = 0.0;
    key_ = 0;
    modifiers_ = 0;
    button_ = 0;
    wheelSteps_ = 0.0;
    cameraIndex_ = -1;
    sceneWidget_ = nullptr;
}


SceneWidgetEvent::SceneWidgetEvent(const SceneWidgetEvent& org)
    : point_(org.point_)
{
    x_ = org.x_;
    y_ = org.y_;
    pixelSizeRatio_ = org.pixelSizeRatio_;
    key_ = org.key_;
    modifiers_ = org.modifiers_;
    button_ = org.button_;
    wheelSteps_ = org.wheelSteps_;
    cameraIndex_ = org.cameraIndex_;
    sceneWidget_ = org.sceneWidget_;
}


const SgCamera* SceneWidgetEvent::camera() const
{
    if(cameraPath_.empty()){
        return nullptr;
    }
    return dynamic_cast<SgCamera*>(cameraPath_.back());
}


const Isometry3& SceneWidgetEvent::cameraPosition() const
{
    if(cameraIndex_ >= 0){
        return sceneWidget_->renderer()->cameraPosition(cameraIndex_);
    } else {
        static const Isometry3 I = Isometry3::Identity();
        return I;
    }
}


const Isometry3& SceneWidgetEvent::currentCameraPosition() const
{
    return cameraPosition();
}


bool SceneWidgetEvent::getRay(Vector3& out_origin, Vector3& out_direction) const
{
    auto T = cameraPosition();
    Vector3 p_near;
    sceneWidget_->unproject(x_, y_, 0.0 /* near plane */, p_near);
    auto camera = sceneWidget_->renderer()->currentCamera();

    if(dynamic_cast<SgPerspectiveCamera*>(camera)){
        out_origin = T.translation();
        out_direction = (p_near - out_origin).normalized();
        return true;

    } else if(dynamic_cast<SgOrthographicCamera*>(camera)){
        out_origin = p_near;
        out_direction = -T.linear().col(2); // -Z
        return true;
    }
    
    return false;
}


void SceneWidgetEvent::updateIndicator(const std::string& text) const
{
    sceneWidget_->updateIndicator(text);
}
