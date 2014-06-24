/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_WIDGET_EDITABLE_H
#define CNOID_BASE_SCENE_WIDGET_EDITABLE_H

#include <cnoid/SceneGraph>
#include "exportdecl.h"

namespace cnoid {

class SceneWidget;
class MenuManager;
    
class CNOID_EXPORT SceneWidgetEvent
{
public:
    SceneWidgetEvent(const SceneWidgetEvent& org);
        
    const Vector3& point() const { return point_; }

    const SgNodePath& nodePath() const { return nodePath_; }

    // OpenGL viewport coordinate
    double x() const { return x_; }
    double y() const { return y_; }

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

    const SgNodePath& cameraPath() const { return cameraPath_; }

    const Affine3& currentCameraPosition() const;

    SceneWidget* sceneWidget() const { return sceneWidget_; }

    void updateIndicator(const std::string& message) const;
        
private:
    int key_;
    int button_;
    int modifiers_;
    Vector3 point_;
    double x_;
    double y_;
    double wheelSteps_;
    SgNodePath nodePath_;
    SgNodePath cameraPath_;
        
    mutable SceneWidget* sceneWidget_;

    SceneWidgetEvent();
    SceneWidgetEvent& operator=(const SceneWidgetEvent& org); // disabled

    friend class SceneWidgetImpl;
};


class CNOID_EXPORT SceneWidgetEditable
{
public:
    virtual bool onKeyPressEvent(const SceneWidgetEvent& event);
    virtual bool onKeyReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onDoubleClickEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event);
    virtual bool onScrollEvent(const SceneWidgetEvent& event);
    virtual void onFocusChanged(const SceneWidgetEvent& event, bool on);
    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager);
    virtual void onSceneModeChanged(const SceneWidgetEvent& event);
    virtual bool onUndoRequest();
    virtual bool onRedoRequest();
};

}

#endif
