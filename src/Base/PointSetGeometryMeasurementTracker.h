#ifndef CNOID_BASE_POINT_SET_GEOMETRY_MEASUREMENT_TRACKER_H
#define CNOID_BASE_POINT_SET_GEOMETRY_MEASUREMENT_TRACKER_H

#include "GeometryMeasurementTracker.h"
#include "PointSetItem.h"
#include <cnoid/SceneDrawables>
#include <cnoid/ConnectionSet>

namespace cnoid {

class PointSetGeometryMeasurementTracker : public GeometryMeasurementTracker
{
public:
    static void initializeClass();

    PointSetGeometryMeasurementTracker(PointSetItem* pointSetItem);
    virtual void resetMeasurementPoint() override;
    virtual Vector3 getMeasurementPoint() override;
    virtual bool setMeasurementPoint(const SgNodePath& path, const Vector3& localPoint) override;
    virtual bool checkTrackable(const SgNodePath& path) override;
    virtual int getNumShapes() override;
    virtual SgNode* getShape(int index) override;
    virtual Isometry3 getShapePosition(int index) override;
    virtual SignalProxy<void()> sigGeometryChanged() override;

private:
    PointSetItemPtr pointSetItem;
    SgPointSetPtr pointSet;
    Vector3 localPosition;
    Signal<void()> sigGeometryChanged_;
    ScopedConnectionSet connections;
};

}

#endif
