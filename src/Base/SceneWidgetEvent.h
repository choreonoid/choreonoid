/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_WIDGET_EVENT_H
#define CNOID_BASE_SCENE_WIDGET_EVENT_H

#include <cnoid/SceneGraph>
#include "exportdecl.h"

namespace cnoid {

class SceneWidget;
class MenuManager;
    
class CNOID_EXPORT SceneWidgetEvent
{
public:
    SceneWidgetEvent(const SceneWidgetEvent& org);

    enum EventType {
        NoEvent, ModeChange, ButtonPress, ButtonRelease, DoubleClick,
        PointerMove, PointerLeave,  Scroll, KeyPress, KeyRelease,
        FocusChange, ContextMenuRequest };

    EventType type() const { return type_; }

    const Vector3& point() const { return point_; }

    const SgNodePath& nodePath() const { return nodePath_; }

    // OpenGL viewport coordinate
    double x() const { return x_; }
    double y() const { return y_; }

    double pixelSizeRatio() const { return pixelSizeRatio_; }

    /**
       @return a Qt::Key value
    */
    int key() const { return key_; }

    /**
       @return a Qt::MouseButton value
    */
    int button() const { return button_; }

    /**
       @return a Qt::KeyboardModifiers value
    */
    int modifiers() const { return modifiers_; }
        
    double wheelSteps() const { return wheelSteps_; }

    const SgCamera* camera() const;
    int cameraIndex() const { return cameraIndex_; }
    const SgNodePath& cameraPath() const { return cameraPath_; }
    const Isometry3& cameraPosition() const;
    [[deprecated("Use cameraPosition")]]
    const Isometry3& currentCameraPosition() const;
    bool getRay(Vector3& out_origin, Vector3& out_direction) const;

    SceneWidget* sceneWidget() const { return sceneWidget_; }

    MenuManager* contextMenu() { return contextMenu_; }

    void updateIndicator(const std::string& message) const;
        
private:
    EventType type_;
    int key_;
    int button_;
    int modifiers_;
    Vector3 point_;
    double x_;
    double y_;
    double pixelSizeRatio_;
    double wheelSteps_;
    SgNodePath nodePath_;
    int cameraIndex_;
    SgNodePath cameraPath_;
    mutable SceneWidget* sceneWidget_;
    MenuManager* contextMenu_;

    SceneWidgetEvent();
    SceneWidgetEvent& operator=(const SceneWidgetEvent& org); // disabled

    friend class SceneWidget;
};

}

#endif
