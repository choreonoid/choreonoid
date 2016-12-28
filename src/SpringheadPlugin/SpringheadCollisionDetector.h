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
    virtual const char* name() const;
    virtual CollisionDetectorPtr clone() const;
    virtual void clearGeometries();
    virtual int  numGeometries() const;
    virtual int  addGeometry(SgNodePtr geometry);
    virtual void setGeometryStatic(int geometryId, bool isStatic = true);
    virtual bool enableGeometryCache(bool on);
    virtual void clearGeometryCache(SgNodePtr geometry);
    virtual void clearAllGeometryCaches();
    virtual void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2);
    virtual bool makeReady();
    virtual void updatePosition(int geometryId, const Position& position);
    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback);

private:
    SpringheadCollisionDetectorImpl* impl;
};

typedef std::shared_ptr<SpringheadCollisionDetector> SpringheadCollisionDetectorPtr;
}

#endif
