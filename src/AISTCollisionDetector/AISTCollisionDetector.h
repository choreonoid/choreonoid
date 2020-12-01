/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_AIST_COLLISION_DETECTOR_AIST_COLLISION_DETECTOR_H
#define CNOID_AIST_COLLISION_DETECTOR_AIST_COLLISION_DETECTOR_H

#include <cnoid/CollisionDetector>
#include "exportdecl.h"

namespace cnoid {

class AISTCollisionDetectorImpl;

class CNOID_EXPORT AISTCollisionDetector : public CollisionDetector, public CollisionDetectorDistanceAPI
{
public:
    AISTCollisionDetector();
    virtual ~AISTCollisionDetector();
    virtual const char* name() const override;
    virtual CollisionDetector* clone() const override;
    virtual void clearGeometries() override;
    virtual int numGeometries() const override;
    virtual stdx::optional<GeometryHandle> addGeometry(SgNode* geometry) override;
    virtual void setCustomObject(GeometryHandle geometry, Referenced* object) override;
    virtual void setGeometryStatic(GeometryHandle geometry, bool isStatic = true) override;
    virtual void ignoreGeometryPair(GeometryHandle geometry1, GeometryHandle geometry2, bool ignore = true) override;
    virtual bool makeReady() override;
    virtual void updatePosition(GeometryHandle geometry, const Isometry3& position) override;
    virtual void updatePositions(std::function<void(Referenced* object, Isometry3*& out_Position)> positionQuery) override;
    virtual void detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback) override;

    // CollisionDetectorDistanceAPI
    virtual double detectDistance(GeometryHandle geometry1, GeometryHandle geometry2, Vector3& out_point1, Vector3& out_point2) override;

    // experimental
    void setNumThreads(int n);

private:
    AISTCollisionDetectorImpl* impl;
};

typedef ref_ptr<AISTCollisionDetector> AISTCollisionDetectorPtr;

}

#endif
