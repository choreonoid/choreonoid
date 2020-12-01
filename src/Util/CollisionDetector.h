/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_COLLISION_DETECTOR_H
#define CNOID_UTIL_COLLISION_DETECTOR_H

#include "Collision.h"
#include "Referenced.h"
#include <cnoid/stdx/optional>
#include <cstdint>
#include "exportdecl.h"

namespace cnoid {

class SgNode;
class CollisionPair;

class CNOID_EXPORT CollisionDetector : public Referenced
{
public:
    typedef intptr_t GeometryHandle;

    static bool registerFactory(const std::string& name, std::function<CollisionDetector*()> factory);
    static int numFactories();
    static std::string factoryName(int factoryIndex);
    static int factoryIndex(const std::string& name);
    static CollisionDetector* create(int factoryIndex);
        
    virtual ~CollisionDetector();
    virtual const char* name() const = 0;

    /**
       \note The geometries and the non interfarence pairs of them
       are not copied to the clone object. That is same as the state
       after calling clearGeometries();
    */
    virtual CollisionDetector* clone() const = 0;
        
    virtual void clearGeometries() = 0;
    virtual int numGeometries() const = 0;

    /**
       \return A handle of the geometry in the collision detector
    */
    virtual stdx::optional<GeometryHandle> addGeometry(SgNode* geometry) = 0;
    virtual void setCustomObject(GeometryHandle geometry, Referenced* object) = 0;
    virtual void setGeometryStatic(GeometryHandle geometry, bool isStatic = true) = 0;
    virtual void ignoreGeometryPair(GeometryHandle geometry1, GeometryHandle geometry2, bool ignore = true) = 0;
    virtual bool makeReady() = 0;
    
    virtual void updatePosition(GeometryHandle geometry, const Isometry3& position) = 0;
    virtual void updatePositions(
        std::function<void(Referenced* object, Isometry3*& out_position)> positionQuery) = 0;

    virtual void detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback) = 0;
};

typedef ref_ptr<CollisionDetector> CollisionDetectorPtr;


class CollisionDetectorDistanceAPI
{
public:
    virtual double detectDistance(
        CollisionDetector::GeometryHandle geometry1, CollisionDetector::GeometryHandle geometry2,
        Vector3& out_point1, Vector3& out_point2) = 0;
};


class CollisionPair
{
    typedef CollisionDetector::GeometryHandle GeometryHandle;
    GeometryHandle geometries_[2];
    Referenced* objects_[2];
    CollisionArray collisions_;

public:
    CollisionPair() { }
    CollisionPair(GeometryHandle geometry1, GeometryHandle geometry2){
        geometries_[0] = geometry1;
        geometries_[1] = geometry2;
    }
    CollisionPair(GeometryHandle geometry1, Referenced* object1, GeometryHandle geometry2, Referenced* object2){
        geometries_[0] = geometry1;
        geometries_[1] = geometry2;
        objects_[0] = object1;
        objects_[1] = object2;
    }
    GeometryHandle& geometry(int i) { return geometries_[i]; };
    const GeometryHandle geometry(int i) const { return geometries_[i]; };
    const GeometryHandle* geometries() const { return geometries_; }
    Referenced*& object(int i){ return objects_[i]; };
    Referenced* object(int i) const { return objects_[i]; };
    CollisionArray& collisions() { return collisions_; }
    const CollisionArray& collisions() const { return collisions_; }
    void addCollision(const Collision& c){ collisions_.push_back(c); }
    Collision& newCollision() { collisions_.resize(collisions_.size() + 1); return collisions_.back(); }
    void clearCollisions(){ collisions_.clear(); }
    bool empty() const { return collisions_.empty(); }
    int numCollisions() const { return collisions_.size(); }
};

}

#endif
