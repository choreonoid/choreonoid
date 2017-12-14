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
    virtual int  numGeometries() const override;
    virtual int  addGeometry(SgNodePtr geometry) override;
    virtual void setGeometryStatic(int geometryId, bool isStatic = true) override;
    virtual bool enableGeometryCache(bool on) override;
    virtual void clearGeometryCache(SgNode* geometry) override;
    virtual void clearAllGeometryCaches() override;
    virtual void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2) override;
    virtual bool makeReady() override;
    virtual void updatePosition(int geometryId, const Position& position) override;
    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback) override;

private:
    SpringheadCollisionDetectorImpl* impl;
};

typedef ref_ptr<SpringheadCollisionDetector> SpringheadCollisionDetectorPtr;
}

#endif
