#ifndef CNOID_BASE_SCENE_POINT_SELECTION_MODE_H
#define CNOID_BASE_SCENE_POINT_SELECTION_MODE_H

#include "SceneWidgetEditable.h"
#include <cnoid/EigenTypes>
#include <vector>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ScenePointSelectionMode : public SceneWidgetEditable
{
    class Impl;
    
public:
    ScenePointSelectionMode();
    ScenePointSelectionMode(const ScenePointSelectionMode& org) = delete;
    virtual ~ScenePointSelectionMode();

    void setCustomModeId(int id);

    std::vector<Vector3f> getSelectedPoints() const;
    void clearSelection();

    class PointInfo : public Referenced
    {
    public:
        PointInfo();
        bool operator==(const PointInfo& rhs) const;
        const SgNodePath& path() const { return *path_; }
        const Vector3f& position() const { return position_; }
        /**
           \return The vertex index when the point is a mesh vertex.
           Otherwise, -1 is returned.
        */
        int vertexIndex() const { return vertexIndex_; }

    private:
        std::shared_ptr<SgNodePath> path_;
        Vector3f position_;
        int vertexIndex_;

        friend class ScenePointSelectionMode::Impl;
    };
    
    typedef ref_ptr<PointInfo> PointInfoPtr;

    PointInfo* highlightedPoint();

protected:
    virtual std::vector<SgNode*> getTargetSceneNodes(const SceneWidgetEvent& event);

    //virtual void onSelectionModeActivated(const SceneWidgetEvent& event);
    //virtual void onSelectionModeDeactivated(const SceneWidgetEvent& event);

    // SceneWidgetEditable functions
    virtual void onSceneModeChanged(const SceneWidgetEvent& event) override;
    virtual bool onButtonPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event) override;
    virtual bool onDoubleClickEvent(const SceneWidgetEvent& event) override;
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event) override;
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event) override;
    virtual bool onKeyPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onKeyReleaseEvent(const SceneWidgetEvent& event) override;
    virtual bool onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menu) override;
    virtual bool onUndoRequest() override;
    virtual bool onRedoRequest() override;

private:
    Impl* impl;
};

}

#endif
