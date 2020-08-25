#ifndef CNOID_BASE_SCENE_WAYPOINT_EDIT_MANAGER_H
#define CNOID_BASE_SCENE_WAYPOINT_EDIT_MANAGER_H

#include "SceneWidgetEditable.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneWaypointEditManager : public SceneWidgetEditable
{
public:
    SceneWaypointEditManager();
    SceneWaypointEditManager(const SceneWaypointEditManager& org) = delete;
    virtual ~SceneWaypointEditManager();

    void setCustomModeId(int id);

    // SceneWidgetEditable functions
    virtual void onSceneModeChanged(const SceneWidgetEvent& event) override;
    virtual bool onButtonPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event) override;
    virtual bool onDoubleClickEvent(const SceneWidgetEvent& event) override;
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event) override;
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event) override;
    virtual bool onKeyPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onKeyReleaseEvent(const SceneWidgetEvent& event) override;
    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager) override;
    virtual bool onUndoRequest() override;
    virtual bool onRedoRequest() override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
