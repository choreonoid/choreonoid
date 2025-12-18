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
    std::optional<GeometryHandle> findGeometryHandle(Link* link);

    typedef std::function<Referenced*(Link* link, GeometryHandle geometry)> LinkAssociatedObjectFunc;
    void setLinkAssociatedObjectFunction(LinkAssociatedObjectFunc func);

    bool isMultiplexBodySupported() const;
    bool isMultiplexBodySupportEnabled() const;
    void setMultiplexBodySupportEnabled(bool on);
    
    void addBody(Body* body, bool isSelfCollisionDetectionEnabled = true, int groupId = 0);
    std::optional<GeometryHandle> addLink(Link* link, int groupId = 0);

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

    bool removeBody(Body* body);

    bool hasBodies() const;

    /**
       \note Geometry handle map must be enabled to use this function.
    */
    void setLinksInAttachmentIgnored(Link* attachedLink, Link* parentLink, bool ignored);
    
    bool makeReady(bool doForce = false);

    void updatePositions();
    void updatePositions(std::function<void(Referenced* object, Isometry3*& out_position)> positionQuery);

    /**
     * @brief Detect collisions between all registered bodies
     * @param callback Function called for each detected collision pair.
     *                 Return false to continue detection, true to stop early.
     * @return true if the callback returned true (early termination),
     *         false if all pairs were checked
     */
    bool detectCollisions(std::function<bool(const CollisionPair& collisionPair)> callback);

    /**
     * @brief Detect collisions between a specific link and all others
     * @param link The link to check for collisions
     * @param callback Function called for each detected collision pair.
     *                 Return false to continue detection, true to stop early.
     * @return true if the callback returned true (early termination),
     *         false if all pairs were checked
     * @note Geometry handle map must be enabled to use this function
     */
    bool detectCollisions(Link* link, std::function<bool(const CollisionPair& collisionPair)> callback);

    [[deprecated("Use setGeometryHandleMapEnabled.")]]
    void enableGeometryHandleMap(bool on);

private:
    class Impl;
    Impl* impl;
};

}

#endif
