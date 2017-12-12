/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_ODEPLUGIN_ODE_COLLISION_DETECTOR_H
#define CNOID_ODEPLUGIN_ODE_COLLISION_DETECTOR_H

#include <cnoid/CollisionDetector>

#ifdef GAZEBO_ODE
#define ODECollisionDetector GazeboODECollisionDetector
#endif

namespace cnoid {

class ODECollisionDetectorImpl;

class ODECollisionDetector : public CollisionDetector
{
public:
    ODECollisionDetector();
    virtual ~ODECollisionDetector();
    virtual const char* name() const override;
    virtual CollisionDetector* clone() const override;
    virtual void clearGeometries() override;
    virtual int numGeometries() const override;
    virtual int addGeometry(SgNode* geometry) override;
    virtual void setGeometryStatic(int geometryId, bool isStatic = true) override;
    virtual bool enableGeometryCache(bool on) override;
    virtual void clearGeometryCache(SgNode* geometry) override;
    virtual void clearAllGeometryCaches() override;
    virtual void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2) override;
    virtual bool makeReady() override;
    virtual void updatePosition(int geometryId, const Position& position) override;
    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback) override;

private:
    ODECollisionDetectorImpl* impl;
};

typedef ref_ptr<ODECollisionDetector> ODECollisionDetectorPtr;

}

#endif
