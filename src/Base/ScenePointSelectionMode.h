#ifndef CNOID_BASE_SCENE_POINT_SELECTION_MODE_H
#define CNOID_BASE_SCENE_POINT_SELECTION_MODE_H

#include "SceneWidgetEventHandler.h"
#include <cnoid/EigenTypes>
#include <cnoid/Signal>
#include <vector>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ScenePointSelectionMode : public SceneWidgetEventHandler
{
    class Impl;
    
public:
    ScenePointSelectionMode();
    ScenePointSelectionMode(const ScenePointSelectionMode& org) = delete;
    virtual ~ScenePointSelectionMode();

    void setCustomModeId(int id);
    int customModeId() const;

    enum SubMode {
        SurfacePointMode,
        MeshVertexMode
    };
    void setSubMode(int mode);
    int subMode() const;
    
    void setNormalDetectionEnabled(bool on);
    void setControlModifierEnabled(bool on);
    void updateTargetSceneNodes();

    class PointInfo : public Referenced
    {
    public:
        PointInfo();

        bool operator==(const PointInfo& rhs) const = delete;
        bool hasSameVertexWith(const PointInfo& point) const;

        const SgNodePath& path() const { return *path_; }

        /**
           \return The vertex index when the point is a mesh vertex.
           Otherwise, -1 is returned.
        */
        int vertexIndex() const { return vertexIndex_; }

        /**
           \return The trialngle vertex index when the point is a mesh vertex.
           Otherwise, -1 is returned.
        */
        int triangleVertexIndex() const { return triangleVertexIndex_; }
        
        const Vector3& position() const { return position_; }
        const Vector3& normal() const { return normal_; }
        bool hasNormal() const { return hasNormal_; }

    private:
        std::shared_ptr<SgNodePath> path_;
        int vertexIndex_;
        int triangleVertexIndex_;
        Vector3 position_;
        Vector3 normal_;
        bool hasNormal_;

        friend class ScenePointSelectionMode::Impl;
    };
    
    typedef ref_ptr<PointInfo> PointInfoPtr;

    PointInfo* highlightedPoint();

    const std::vector<PointInfoPtr>& selectedPoints() const;
    void clearSelection();

    void setHighlightedPointColor(const Vector3f& coloir);
    void setSelectedPointColor(const Vector3f& coloir);

    SignalProxy<void(const std::vector<PointInfoPtr>& additionalPoints)> sigPointSelectionAdded();

protected:
    virtual std::vector<SgNode*> getTargetSceneNodes(SceneWidget* sceneWidget);
    virtual void onPointSelectionAdded(const std::vector<PointInfoPtr>& additionalPoints);

    //virtual void onSelectionModeActivated(SceneWidgetEvent* event);
    //virtual void onSelectionModeDeactivated(SceneWidgetEvent* event);

    // SceneWidgetEventHandler functions
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;

private:
    Impl* impl;
};

}

#endif
