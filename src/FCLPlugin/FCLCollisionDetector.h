/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_FCLPLUGIN__FCL_COLLISION_DETECTOR_H_INCLUDED
#define CNOID_FCLPLUGIN__FCL_COLLISION_DETECTOR_H_INCLUDED

#include <cnoid/CollisionDetector>
#include <memory>
namespace cnoid {

class FCLCollisionDetectorImpl;

class FCLCollisionDetector : public CollisionDetector
{
public:
    FCLCollisionDetector();
    virtual ~FCLCollisionDetector();
    virtual const char* name() const;
    virtual CollisionDetector* clone() const;
    virtual void clearGeometries();
    virtual int numGeometries() const;
    virtual int addGeometry(SgNode* geometry);
    virtual void setGeometryStatic(int geometryId, bool isStatic = true);
    virtual bool enableGeometryCache(bool on);
    virtual void clearGeometryCache(SgNode* geometry);
    virtual void clearAllGeometryCaches();
    virtual void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2);
    virtual bool makeReady();
    virtual void updatePosition(int geometryId, const Position& position);
    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback);

private:
    FCLCollisionDetectorImpl* impl;
};

typedef ref_ptr<FCLCollisionDetector> FCLCollisionDetectorPtr;
}

#endif
