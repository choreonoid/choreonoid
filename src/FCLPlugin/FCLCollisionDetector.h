#ifndef CNOID_FCLPLUGIN_FCL_COLLISION_DETECTOR_H
#define CNOID_FCLPLUGIN_FCL_COLLISION_DETECTOR_H

#include <cnoid/CollisionDetector>

namespace cnoid {

class FCLCollisionDetector : public CollisionDetector
{
public:
    FCLCollisionDetector();
    virtual ~FCLCollisionDetector();
    virtual const char* name() const override;
    virtual CollisionDetector* clone() const override;
    virtual void clearGeometries() override;
    virtual int numGeometries() const override;
    virtual stdx::optional<GeometryHandle> addGeometry(SgNode* geometry) override;
    virtual void setCustomObject(GeometryHandle geometry, Referenced* object) override;
    virtual void setGeometryStatic(GeometryHandle geometry, bool isStatic = true) override;
    virtual void ignoreGeometryPair(GeometryHandle geometry1, GeometryHandle geometry2, bool ignore = true) override;
    virtual bool makeReady() override;
    virtual void updatePosition(GeometryHandle geometry, const Isometry3& position) override;
    virtual void updatePositions(
        std::function<void(Referenced* object, Isometry3*& out_position)> positionQuery) override;
    virtual void detectCollisions(std::function<void(const CollisionPair&)> callback) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<FCLCollisionDetector> FCLCollisionDetectorPtr;

}

#endif
