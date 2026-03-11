#ifndef CNOID_PHYSX_PLUGIN_PX_CONTINUOUS_TRACK_SIMULATOR_H
#define CNOID_PHYSX_PLUGIN_PX_CONTINUOUS_TRACK_SIMULATOR_H

#include "PxContinuousTrack.h"
#include <PxPhysicsAPI.h>
#include <cnoid/EigenTypes>
#include <vector>
#include <memory>

namespace cnoid {

class Link;
class Body;
class PhysxBody;

class PxContinuousTrackHandler
{
public:
    PxContinuousTrackHandler(PxContinuousTrack* device);
    ~PxContinuousTrackHandler();

    bool initialize(
        Body* body,
        physx::PxPhysics* physics,
        physx::PxArticulationReducedCoordinate* articulation,
        physx::PxArticulationLink* trackArtLink,
        physx::PxArticulationLink* sprocketArtLink,
        physx::PxArticulationLink* idlerArtLink,
        physx::PxMaterial* material,
        int bodyIndex);

    void updateSimulation();
    void updateTrackStates();

    Link* trackLink() const { return trackLink_; }

private:
    struct LinearSegment {
        physx::PxArticulationLink* artLink;
        physx::PxArticulationJointReducedCoordinate* joint;
        double jointPosition;
        // Direction: +1 for upper (sprocket->idler), -1 for lower (idler->sprocket)
        double direction;
        // Offset from track origin to segment center (in track local frame)
        Vector3 localOrigin;
        // Segment sliding axis in track local frame
        Vector3 slideAxis;
        // Length of the linear segment
        double length;
        // Number of grousers on this segment
        int numGrousers;
        physx::PxShape* plateShape;
    };

    PxContinuousTrack* device_;
    Link* trackLink_;
    Link* sprocketLink_;

    physx::PxArticulationReducedCoordinate* articulation_;
    physx::PxArticulationLink* trackArtLink_;
    physx::PxArticulationLink* sprocketArtLink_;
    physx::PxArticulationLink* idlerArtLink_;

    LinearSegment upperSegment_;
    LinearSegment lowerSegment_;

    // Wheel grouser info for visualization
    struct WheelGrouserSet {
        physx::PxArticulationLink* wheelLink;
        double wheelRadius;
        int numGrousers;         // half-arc count (visible, for rendering)
        int numPhysicsGrousers;  // full-circle count (shapes on wheel link)
        double angularOffset;    // Starting angular offset
        double angleStep;        // 2*PI/numPhysicsGrousers
    };
    WheelGrouserSet sprocketGrousers_;
    WheelGrouserSet idlerGrousers_;

    int bodyIndex_;

    physx::PxArticulationMimicJoint* sprocketMimicJoint_;
    physx::PxArticulationMimicJoint* idlerMimicJoint_;
    physx::PxArticulationMimicJoint* upperSegmentMimicJoint_;

    void addGrouserShapesToWheel(
        physx::PxPhysics* physics,
        physx::PxArticulationLink* wheelLink,
        double wheelRadius,
        physx::PxMaterial* material);

    void addBeltShapesToWheel(
        physx::PxPhysics* physics,
        physx::PxArticulationLink* wheelLink,
        double wheelRadius,
        int numBeltSegments,
        physx::PxMaterial* material);

    bool resetAllJointPositions();

    void addSegmentShoePositions(const LinearSegment& segment, const Isometry3& trackT_inv);
    void addWheelShoePositions(
        const WheelGrouserSet& wheelGrousers,
        bool isOuterPositive,
        const Isometry3& trackT_inv);

    bool createLinearSegment(
        physx::PxPhysics* physics,
        physx::PxArticulationReducedCoordinate* articulation,
        physx::PxArticulationLink* parentLink,
        physx::PxMaterial* material,
        LinearSegment& segment,
        const Vector3& localOrigin,
        const Vector3& slideAxis,
        double segmentLength,
        double direction,
        double segmentMass);
};


class PxContinuousTrackSimulator
{
public:
    PxContinuousTrackSimulator();
    ~PxContinuousTrackSimulator();

    // Scan PxContinuousTrack devices on a body and create handlers.
    // Called from PhysxBody::createPhysxObjects().
    int setupTrackHandlers(PhysxBody* physxBody);

    // Generate initial track states for all handlers
    void initializeTrackStates();

    // Update simulation for all handlers
    void updateSimulation();

    // Update track states for all handlers
    void updateTrackStates();

    bool empty() const;

private:
    struct TrackHandlerEntry {
        std::unique_ptr<PxContinuousTrackHandler> handler;
    };
    std::vector<TrackHandlerEntry> handlers_;
};

}

#endif
