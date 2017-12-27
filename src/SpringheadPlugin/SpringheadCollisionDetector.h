/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_SpringheadPLUGIN__Springhead_COLLISION_DETECTOR_H_INCLUDED
#define CNOID_SpringheadPLUGIN__Springhead_COLLISION_DETECTOR_H_INCLUDED

#include <cnoid/CollisionDetector>

namespace cnoid {

class SpringheadCollisionDetectorImpl;

class SpringheadCollisionDetector : public CollisionDetector
{
public:
    SpringheadCollisionDetector();
    virtual ~SpringheadCollisionDetector();

	virtual const char* name() const override;
    virtual CollisionDetector* clone() const override;
    virtual void clearGeometries() override;
    virtual int numGeometries() const override;
    virtual boost::optional<GeometryHandle> addGeometry(SgNode* geometry) override;
    virtual void setCustomObject(GeometryHandle geometry, Referenced* object) override;
    virtual void setGeometryStatic(GeometryHandle geometry, bool isStatic = true) override;
    virtual void setNonInterfarenceGeometyrPair(GeometryHandle geometry1, GeometryHandle geometry2) override;
    virtual bool makeReady() override;
    virtual void updatePosition(GeometryHandle geometry, const Position& position) override;
    virtual void updatePositions(std::function<void(Referenced* object, Position*& out_Position)> positionQuery) override;
    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback) override;

private:
    SpringheadCollisionDetectorImpl* impl;
};

typedef ref_ptr<SpringheadCollisionDetector> SpringheadCollisionDetectorPtr;
}

#endif
