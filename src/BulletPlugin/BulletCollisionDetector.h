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
    BulletCollisionDetectorImpl* impl;
};

typedef ref_ptr<BulletCollisionDetector> BulletCollisionDetectorPtr;

}

#endif
