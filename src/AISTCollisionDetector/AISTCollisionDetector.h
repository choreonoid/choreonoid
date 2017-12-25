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

class CNOID_EXPORT AISTCollisionDetector : public CollisionDetector
{
public:
    AISTCollisionDetector();
    virtual ~AISTCollisionDetector();
    virtual const char* name() const override;
    virtual CollisionDetector* clone() const override;
    virtual void clearGeometries() override;
    virtual int numGeometries() const override;
    virtual int addGeometry(SgNode* geometry, Referenced* object) override;
    virtual void setGeometryStatic(int geometryId, bool isStatic = true) override;
    virtual void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2) override;
    virtual bool makeReady() override;
    virtual void updatePosition(int geometryId, const Position& position) override;
    virtual void updatePositions(std::function<void(Referenced* object, Position*& out_Position)> positionQuery) override;
    virtual void detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback) override;

    virtual bool isFindClosestPointsAvailable() const override;
    virtual double findClosestPoints(int geometryId1, int geometryId2, Vector3& out_point1, Vector3& out_point2) override;

    // experimental
    void setNumThreads(int n);

    //! \deprecated
    int geometryPairId(int geometryId1, int geometryId2) const;
    //! \deprecated
    double findClosestPoints(int geometryPairId, Vector3& out_point1, Vector3& out_point2);

private:
    AISTCollisionDetectorImpl* impl;
};

typedef ref_ptr<AISTCollisionDetector> AISTCollisionDetectorPtr;

}

#endif
