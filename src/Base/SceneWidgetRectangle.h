/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_WIDGET_RECTANGLE_H
#define CNOID_BASE_SCENE_WIDGET_RECTANGLE_H

#include "SceneWidgetEditable.h"
#include <QCursor>
#include "exportdecl.h"

namespace cnoid {

class SceneWidgetRectangleImpl;

class CNOID_EXPORT SceneWidgetRectangle : public SceneWidgetEditable
{
public:
    SceneWidgetRectangle(SceneWidget* sceneWidget);
    ~SceneWidgetRectangle();

    void setEditModeCursor(QCursor cursor);

    class CNOID_EXPORT Region {
      public:
        Region();
        Region(int numSurroundingPlanes);
        Region(const Region& org);
        Region& operator=(const Region& org);
        int numSurroundingPlanes() const;
        void setNumSurroundingPlanes(int n);
        void addSurroundingPlane(const Vector3& normal, const Vector3& point);
        Vector3& normal(int index);
        const Vector3& normal(int index) const;
        Vector3& point(int index);
        const Vector3& point(int index) const;
      private:
        void* impl;
    };

    const Region& region() const;
    SignalProxy<void(const SceneWidgetRectangle::Region& region)> sigRegionFixed();

    virtual void onSceneModeChanged(const SceneWidgetEvent& event);
    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);

private:
    SceneWidgetRectangleImpl* impl;
};

}

#endif
