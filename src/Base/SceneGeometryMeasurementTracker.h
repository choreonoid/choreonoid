#ifndef CNOID_BASE_SCENE_GEOMETRY_MEASUREMENT_TRACKER_H
#define CNOID_BASE_SCENE_GEOMETRY_MEASUREMENT_TRACKER_H

#include "GeometryMeasurementTracker.h"
#include "SceneItem.h"
#include <cnoid/ConnectionSet>

namespace cnoid {

class SceneGeometryMeasurementTracker : public GeometryMeasurementTracker
{
public:
    static void initializeClass();

    SceneGeometryMeasurementTracker(SceneItem* sceneItem);
    virtual void resetMeasurementPoint() override;
    virtual Vector3 getMeasurementPoint() override;
    virtual bool setMeasurementPoint(const SgNodePath& path, const Vector3& localPoint) override;
    virtual bool checkTrackable(const SgNodePath& path) override;
    virtual int getNumShapes() override;
    virtual SgNode* getShape(int index) override;
    virtual Isometry3 getShapePosition(int index) override;
    virtual SignalProxy<void()> sigGeometryChanged() override;

private:
    SceneItemPtr sceneItem;
    SgNodePath scenePath;
    Vector3 localPosition;
    Signal<void()> sigGeometryChanged_;
    ScopedConnectionSet connections;
};

}

#endif
