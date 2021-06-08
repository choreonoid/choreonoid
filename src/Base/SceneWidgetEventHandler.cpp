/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidgetEventHandler.h"

using namespace cnoid;


void SceneWidgetEventHandler::onSceneModeChanged(SceneWidgetEvent* event)
{
    onSceneModeChanged(*event);
}


bool SceneWidgetEventHandler::onButtonPressEvent(SceneWidgetEvent* event)
{
    return onButtonPressEvent(*event);
}


bool SceneWidgetEventHandler::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    return onButtonReleaseEvent(*event);
}


bool SceneWidgetEventHandler::onDoubleClickEvent(SceneWidgetEvent* event)
{
    return onDoubleClickEvent(*event);
}


bool SceneWidgetEventHandler::onPointerMoveEvent(SceneWidgetEvent* event)
{
    return onPointerMoveEvent(*event);
}


void SceneWidgetEventHandler::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    onPointerLeaveEvent(*event);
}


bool SceneWidgetEventHandler::onScrollEvent(SceneWidgetEvent* event)
{
    return onScrollEvent(*event);
}


bool SceneWidgetEventHandler::onKeyPressEvent(SceneWidgetEvent* event)
{
    return onKeyPressEvent(*event);
}


bool SceneWidgetEventHandler::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    return onKeyReleaseEvent(*event);
}


void SceneWidgetEventHandler::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    onFocusChanged(*event, on);
}


bool SceneWidgetEventHandler::onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menu)
{
    return onContextMenuRequest(*event, *menu);
}


void SceneWidgetEventHandler::onSceneModeChanged(const SceneWidgetEvent&)
{

}


bool SceneWidgetEventHandler::onButtonPressEvent(const SceneWidgetEvent&)
{
    return false;
}


bool SceneWidgetEventHandler::onButtonReleaseEvent(const SceneWidgetEvent&)
{
    return false;
}


bool SceneWidgetEventHandler::onDoubleClickEvent(const SceneWidgetEvent&)
{
    return false;
}


bool SceneWidgetEventHandler::onPointerMoveEvent(const SceneWidgetEvent&)
{
    return false;
}


void SceneWidgetEventHandler::onPointerLeaveEvent(const SceneWidgetEvent&)
{

}


bool SceneWidgetEventHandler::onScrollEvent(const SceneWidgetEvent&)
{
    return false;
}


bool SceneWidgetEventHandler::onKeyPressEvent(const SceneWidgetEvent&)
{
    return false;
}


bool SceneWidgetEventHandler::onKeyReleaseEvent(const SceneWidgetEvent&)
{
    return false;
}


void SceneWidgetEventHandler::onFocusChanged(const SceneWidgetEvent&, bool)
{

}


bool SceneWidgetEventHandler::onContextMenuRequest(const SceneWidgetEvent&, MenuManager&)
{
    return false;
}


bool SceneWidgetEventHandler::onUndoRequest()
{
    return false;
}


bool SceneWidgetEventHandler::onRedoRequest()
{
    return false;
}
