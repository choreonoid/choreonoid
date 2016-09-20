/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_COLLISION_DETECTOR_H
#define CNOID_UTIL_COLLISION_DETECTOR_H

#include "Collision.h"
#include "SceneGraph.h"
#include "EigenTypes.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

struct CollisionPair {
    int geometryId[2];
    CollisionArray collisions;
};
typedef std::shared_ptr<CollisionPair> CollisionPairPtr;

class CollisionDetector;
typedef std::shared_ptr<CollisionDetector> CollisionDetectorPtr;

class CNOID_EXPORT CollisionDetector
{
public:
    static bool registerFactory(const std::string& name, std::function<CollisionDetectorPtr()> factory);
    static int numFactories();
    static std::string factoryName(int factoryIndex);
    static int factoryIndex(const std::string& name);
    static CollisionDetectorPtr create(int factoryIndex);
        
    virtual ~CollisionDetector();
    virtual const char* name() const = 0;

    /**
       \note The geometries and the non interfarence pairs of them
       are not copied to the clone object. That is same as the state
       after calling clearGeometries();
    */
    virtual CollisionDetectorPtr clone() const = 0;
        
    virtual bool enableGeometryCache(bool on) = 0;
    virtual void clearGeometryCache(SgNodePtr geometry) = 0;
    virtual void clearAllGeometryCaches() = 0;

    virtual void clearGeometries() = 0;
    /**
       \return id of the geometry
    */
    virtual int numGeometries() const = 0;
    virtual int addGeometry(SgNodePtr geometry) = 0;
    virtual void setGeometryStatic(int geometryId, bool isStatic = true) = 0;
    virtual void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2) = 0;
    virtual bool makeReady() = 0;
    virtual void updatePosition(int geometryId, const Position& position) = 0;

    // Is this faster than the above function?
    //virtual void updatePositions(int begin, int end, const std::vector<Position>& positions) = 0;

    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback) = 0;

    // or
    // virtual void detectCollisions(std::vector<CollisionPair>& out_collisionPairs) = 0;

};

}

#endif
