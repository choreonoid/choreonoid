/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_WIDGET_EVENT_HANDLER_H
#define CNOID_BASE_SCENE_WIDGET_EVENT_HANDLER_H

#include "SceneWidgetEvent.h"
#include "exportdecl.h"

namespace cnoid {

class MenuManager;
    
class CNOID_EXPORT SceneWidgetEventHandler
{
public:
    virtual void onSceneModeChanged(SceneWidgetEvent* event);
    virtual bool onButtonPressEvent(SceneWidgetEvent* event);
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event);
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event);
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event);
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event);
    virtual bool onScrollEvent(SceneWidgetEvent* event);
    virtual bool onKeyPressEvent(SceneWidgetEvent* event);
    virtual bool onKeyReleaseEvent(SceneWidgetEvent* event);
    virtual void onFocusChanged(SceneWidgetEvent* event, bool on);
    virtual bool onContextMenuRequest(SceneWidgetEvent* event);

    // The following functions are deprecated. Override the above functions.
    virtual void onSceneModeChanged(const SceneWidgetEvent& event);
    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onDoubleClickEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event);
    virtual bool onScrollEvent(const SceneWidgetEvent& event);
    virtual bool onKeyPressEvent(const SceneWidgetEvent& event);
    virtual bool onKeyReleaseEvent(const SceneWidgetEvent& event);
    virtual void onFocusChanged(const SceneWidgetEvent& event, bool on);
    virtual bool onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menu);
    virtual bool onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menu);
    // Undo and redo are not handled with SceneWidgetEditable.
    virtual bool onUndoRequest();
    virtual bool onRedoRequest();
};

// For the backward compatibility
typedef SceneWidgetEventHandler SceneWidgetEditable;

}

#endif
