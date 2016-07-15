/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidgetEditable.h"
#include "SceneWidget.h"
#include <cnoid/SceneRenderer>

using namespace cnoid;

SceneWidgetEvent::SceneWidgetEvent()
{
    point_.setZero();
    x_ = 0.0;
    y_ = 0.0;
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


bool SceneWidgetEditable::onKeyPressEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWidgetEditable::onKeyReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWidgetEditable::onButtonPressEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWidgetEditable::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWidgetEditable::onDoubleClickEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWidgetEditable::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    return false;
}


void SceneWidgetEditable::onPointerLeaveEvent(const SceneWidgetEvent& event)
{

}


bool SceneWidgetEditable::onScrollEvent(const SceneWidgetEvent& event)
{
    return false;
}


void SceneWidgetEditable::onFocusChanged(const SceneWidgetEvent& event, bool on)
{

}


void SceneWidgetEditable::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{

}


void SceneWidgetEditable::onSceneModeChanged(const SceneWidgetEvent& event)
{

}


bool SceneWidgetEditable::onUndoRequest()
{
    return false;
}


bool SceneWidgetEditable::onRedoRequest()
{
    return false;
}
