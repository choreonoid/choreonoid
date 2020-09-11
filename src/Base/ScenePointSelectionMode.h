#ifndef CNOID_BASE_SCENE_POINT_SELECTION_MODE_H
#define CNOID_BASE_SCENE_POINT_SELECTION_MODE_H

#include "SceneWidgetEditable.h"
#include <cnoid/EigenTypes>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ScenePointSelectionMode : public SceneWidgetEditable
{
public:
    ScenePointSelectionMode();
    ScenePointSelectionMode(const ScenePointSelectionMode& org) = delete;
    virtual ~ScenePointSelectionMode();

    void setCustomModeId(int id);

    std::vector<Vector3f> getSelectedPoints() const;

protected:
    virtual std::vector<SgNode*> getTargetSceneNodes(const SceneWidgetEvent& event);

    //virtual void onSelectionModeActivated(const SceneWidgetEvent& event);
    //virtual void onSelectionModeDeactivated(const SceneWidgetEvent& event);

private:
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
