/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_WIDGET_EDITABLE_H
#define CNOID_BASE_SCENE_WIDGET_EDITABLE_H

#include "SceneWidgetEvent.h"
#include "exportdecl.h"

namespace cnoid {

class SceneWidgetEvent;
class MenuManager;
    
class CNOID_EXPORT SceneWidgetEditable
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
    virtual bool onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menu);

    // deprecated
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
    virtual bool onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menu);

    [[deprecated("Undo is not handled with SceneWidgetEditable")]]
    virtual bool onUndoRequest();
    [[deprecated("Redo is not handled with SceneWidgetEditable")]]
    virtual bool onRedoRequest();
};

}

#endif
