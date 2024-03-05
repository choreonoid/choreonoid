#include "SceneGeometryMeasurementTracker.h"
#include <cnoid/SceneUtil>

using namespace std;
using namespace cnoid;


static GeometryMeasurementTracker* createSceneGeometryMeasurementTracker(Item* item)
{
    if(auto sceneItem = dynamic_cast<SceneItem*>(item)){
        return new SceneGeometryMeasurementTracker(sceneItem);
    }
    return nullptr;
}


void SceneGeometryMeasurementTracker::initializeClass()
{
    GeometryMeasurementTracker::registerItemTrackerFactory(createSceneGeometryMeasurementTracker);
}


SceneGeometryMeasurementTracker::SceneGeometryMeasurementTracker(SceneItem* sceneItem)
    : sceneItem(sceneItem)
{
    resetMeasurementPoint();

    connections.add(
        sceneItem->topNode()->sigUpdated().connect(
            [this](const SgUpdate& update){
                if(update.hasAction(SgUpdate::Added | SgUpdate::Removed | SgUpdate::GeometryModified)){
                    sigGeometryChanged_();
                }
            }));
}


void SceneGeometryMeasurementTracker::resetMeasurementPoint()
{
    localPosition.setZero();
}


Vector3 SceneGeometryMeasurementTracker::getMeasurementPoint()
{
    Affine3 T = calcTotalTransform(scenePath);
    return T * localPosition;
}


bool SceneGeometryMeasurementTracker::setMeasurementPoint(const SgNodePath& path, const Vector3& point)
{
    scenePath = path;
    Affine3 T = calcTotalTransform(path);
    localPosition = T.inverse() * point;
    return true;
}


bool SceneGeometryMeasurementTracker::checkTrackable(const SgNodePath& /* path */)
{
    return true;
}


int SceneGeometryMeasurementTracker::getNumShapes()
{
    return sceneItem->topNode()->empty() ? 0 : 1;
}


SgNode* SceneGeometryMeasurementTracker::getShape(int index)
{
    auto topNode = sceneItem->topNode();
    if(!topNode->empty() && index == 0){
        return topNode;
    }
    return nullptr;
}


Isometry3 SceneGeometryMeasurementTracker::getShapePosition(int index)
{
    if(index == 0){
        Affine3 T = calcTotalTransform(scenePath);
        if(T.linear().isUnitary()){
            return Isometry3(T.matrix());
        }
    }
    return Isometry3::Identity();
}


SignalProxy<void()> SceneGeometryMeasurementTracker::sigGeometryChanged()
{
    return sigGeometryChanged_;
}
