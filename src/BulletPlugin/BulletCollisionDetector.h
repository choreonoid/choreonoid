/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_BULLETPLUGIN_BULLET_COLLISION_DETECTOR_H
#define CNOID_BULLETPLUGIN_BULLET_COLLISION_DETECTOR_H

#include <cnoid/CollisionDetector>

namespace cnoid {

class BulletCollisionDetectorImpl;

class BulletCollisionDetector : public CollisionDetector
{
public:
    BulletCollisionDetector();
    virtual ~BulletCollisionDetector();
    virtual const char* name() const;
    virtual CollisionDetector* clone() const;
    virtual void clearGeometries();
    virtual int numGeometries() const;
    virtual int addGeometry(SgNode* geometry);
    virtual void setGeometryStatic(int geometryId, bool isStatic = true);
    virtual void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2);
    virtual bool makeReady();
    virtual void updatePosition(int geometryId, const Position& position);
    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback);

private:
    BulletCollisionDetectorImpl* impl;
};

typedef ref_ptr<BulletCollisionDetector> BulletCollisionDetectorPtr;

}

#endif
