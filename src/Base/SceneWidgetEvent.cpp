/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidgetEvent.h"
#include "SceneWidget.h"
#include <cnoid/SceneRenderer>

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
}


const Affine3& SceneWidgetEvent::currentCameraPosition() const
{
    return sceneWidget_->renderer()->currentCameraPosition();
}


void SceneWidgetEvent::updateIndicator(const std::string& text) const
{
    sceneWidget_->updateIndicator(text);
}
