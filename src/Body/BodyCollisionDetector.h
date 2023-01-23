#ifndef CNOID_BODY_BODY_COLLISION_DETECTOR_H
#define CNOID_BODY_BODY_COLLISION_DETECTOR_H

#include <cnoid/CollisionDetector>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;

class CNOID_EXPORT BodyCollisionDetector
{
public:
    BodyCollisionDetector();
    BodyCollisionDetector(CollisionDetector* collisionDetector);
    virtual ~BodyCollisionDetector();

    void setCollisionDetector(CollisionDetector* collisionDetector);
    CollisionDetector* collisionDetector();

    void clearBodies();

    typedef CollisionDetector::GeometryHandle GeometryHandle;

    bool isGeometryHandleMapEnabled() const;
    void setGeometryHandleMapEnabled(bool on);
    stdx::optional<GeometryHandle> findGeometryHandle(Link* link);

    typedef std::function<Referenced*(Link* link, GeometryHandle geometry)> LinkAssociatedObjectFunc;
    void setLinkAssociatedObjectFunction(LinkAssociatedObjectFunc func);
    
    void addBody(Body* body, bool isSelfCollisionDetectionEnabled = true, int groupId = 0);

    //! \note The geometry handle must be enabled for using this function.
    void setGroup(Body* body, int groupId);
    //! \note The geometry handle must be enabled for using this function.
    void setGroup(Link* link, int groupId);

    [[deprecated]]
    void addBody(
        Body* body,
        bool isSelfCollisionDetectionEnabled,
        LinkAssociatedObjectFunc linkAssociatedObjectFunc) {
        setLinkAssociatedObjectFunction(linkAssociatedObjectFunc);
        addBody(body, isSelfCollisionDetectionEnabled, 0);
    }
    
    bool hasBodies() const;
    
    bool makeReady();

    void updatePositions();
    void updatePositions(std::function<void(Referenced* object, Isometry3*& out_position)> positionQuery);

    void detectCollisions(std::function<void(const CollisionPair& collisionPair)> callback);

    //! \note Geometry handle map must be enabled to use this function
    void detectCollisions(Link* link, std::function<void(const CollisionPair& collisionPair)> callback);

    [[deprecated("Use setGeometryHandleMapEnabled.")]]
    void enableGeometryHandleMap(bool on);

private:
    class Impl;
    Impl* impl;
};

}

#endif
