/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidgetEditable.h"

using namespace cnoid;


void SceneWidgetEditable::onSceneModeChanged(SceneWidgetEvent* event)
{
    onSceneModeChanged(*event);
}


bool SceneWidgetEditable::onButtonPressEvent(SceneWidgetEvent* event)
{
    return onButtonPressEvent(*event);
}


bool SceneWidgetEditable::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    return onButtonReleaseEvent(*event);
}


bool SceneWidgetEditable::onDoubleClickEvent(SceneWidgetEvent* event)
{
    return onDoubleClickEvent(*event);
}


bool SceneWidgetEditable::onPointerMoveEvent(SceneWidgetEvent* event)
{
    return onPointerMoveEvent(*event);
}


void SceneWidgetEditable::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    onPointerLeaveEvent(*event);
}


bool SceneWidgetEditable::onScrollEvent(SceneWidgetEvent* event)
{
    return onScrollEvent(*event);
}


bool SceneWidgetEditable::onKeyPressEvent(SceneWidgetEvent* event)
{
    return onKeyPressEvent(*event);
}


bool SceneWidgetEditable::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    return onKeyReleaseEvent(*event);
}


void SceneWidgetEditable::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    onFocusChanged(*event, on);
}


bool SceneWidgetEditable::onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menu)
{
    return onContextMenuRequest(*event, *menu);
}


void SceneWidgetEditable::onSceneModeChanged(const SceneWidgetEvent&)
{

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


bool SceneWidgetEditable::onKeyPressEvent(const SceneWidgetEvent&)
{
    return false;
}


bool SceneWidgetEditable::onKeyReleaseEvent(const SceneWidgetEvent&)
{
    return false;
}


void SceneWidgetEditable::onFocusChanged(const SceneWidgetEvent&, bool)
{

}


bool SceneWidgetEditable::onContextMenuRequest(const SceneWidgetEvent&, MenuManager&)
{
    return false;
}


bool SceneWidgetEditable::onUndoRequest()
{
    return false;
}


bool SceneWidgetEditable::onRedoRequest()
{
    return false;
}
