/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidgetEditable.h"

using namespace cnoid;


bool SceneWidgetEditable::onKeyPressEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWidgetEditable::onKeyReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWidgetEditable::onButtonPressEvent(const SceneWidgetEvent&)
{
    return false;
}


bool SceneWidgetEditable::onButtonReleaseEvent(const SceneWidgetEvent&)
{
    return false;
}


bool SceneWidgetEditable::onDoubleClickEvent(const SceneWidgetEvent&)
{
    return false;
}


bool SceneWidgetEditable::onPointerMoveEvent(const SceneWidgetEvent&)
{
    return false;
}


void SceneWidgetEditable::onPointerLeaveEvent(const SceneWidgetEvent&)
{

}


bool SceneWidgetEditable::onScrollEvent(const SceneWidgetEvent&)
{
    return false;
}


void SceneWidgetEditable::onFocusChanged(const SceneWidgetEvent&, bool)
{

}


void SceneWidgetEditable::onContextMenuRequest(const SceneWidgetEvent&, MenuManager&)
{

}


void SceneWidgetEditable::onSceneModeChanged(const SceneWidgetEvent&)
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
