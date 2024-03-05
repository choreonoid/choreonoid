#include "PointSetGeometryMeasurementTracker.h"

using namespace std;
using namespace cnoid;


static GeometryMeasurementTracker* createPointSetGeometryMeasurementTracker(Item* item)
{
    if(auto pointSetItem = dynamic_cast<PointSetItem*>(item)){
        return new PointSetGeometryMeasurementTracker(pointSetItem);
    }
    return nullptr;
}


void PointSetGeometryMeasurementTracker::initializeClass()
{
    GeometryMeasurementTracker::registerItemTrackerFactory(
        createPointSetGeometryMeasurementTracker);
}


PointSetGeometryMeasurementTracker::PointSetGeometryMeasurementTracker(PointSetItem* pointSetItem)
    : pointSetItem(pointSetItem),
      pointSet(pointSetItem->pointSet())
{
    resetMeasurementPoint();

    connections.add(
        pointSetItem->sigOffsetPositionChanged().connect(
            [this](){ sigGeometryChanged_(); }));

    connections.add(
        pointSet->vertices()->sigUpdated().connect(
            [this](const SgUpdate& update){
                if(update.action() == SgUpdate::GeometryModified){
                    sigGeometryChanged_();
                }
            }));
}


void PointSetGeometryMeasurementTracker::resetMeasurementPoint()
{
    localPosition.setZero();
}


Vector3 PointSetGeometryMeasurementTracker::getMeasurementPoint()
{
    return pointSetItem->offsetPosition() * localPosition;
}


bool PointSetGeometryMeasurementTracker::setMeasurementPoint(const SgNodePath& /* path */, const Vector3& point)
{
    localPosition = pointSetItem->offsetPosition().inverse() * point;
    return true;
}


bool PointSetGeometryMeasurementTracker::checkTrackable(const SgNodePath& /* path */)
{
    return true;
}


int PointSetGeometryMeasurementTracker::getNumShapes()
{
    return pointSet->vertices()->empty() ? 0 : 1;
}


SgNode* PointSetGeometryMeasurementTracker::getShape(int index)
{
    if(!pointSet->vertices()->empty() && index == 0){
        return pointSet;
    }
    return nullptr;
}


Isometry3 PointSetGeometryMeasurementTracker::getShapePosition(int index)
{
    if(index == 0){
        return pointSetItem->offsetPosition();
    }
    return Isometry3::Identity();
}


SignalProxy<void()> PointSetGeometryMeasurementTracker::sigGeometryChanged()
{
    return sigGeometryChanged_;
}
