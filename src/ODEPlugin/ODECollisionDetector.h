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
    virtual stdx::optional<GeometryHandle> addGeometry(SgNode* geometry) override;
    virtual void setCustomObject(GeometryHandle geometry, Referenced* object) override;
    virtual void setGeometryStatic(GeometryHandle geometry, bool isStatic = true) override;
    virtual void setNonInterfarenceGeometyrPair(GeometryHandle geometry1, GeometryHandle geometry2) override;
    virtual bool makeReady() override;
    virtual void updatePosition(GeometryHandle geometry, const Position& position) override;
    virtual void updatePositions(std::function<void(Referenced* object, Position*& out_Position)> positionQuery) override;
    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback) override;

private:
    ODECollisionDetectorImpl* impl;
};

typedef ref_ptr<ODECollisionDetector> ODECollisionDetectorPtr;

}

#endif
