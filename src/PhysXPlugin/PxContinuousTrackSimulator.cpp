#include "PxContinuousTrackSimulator.h"
#include "PhysXSimulatorItemImpl.h"
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Material>
#include <cnoid/EigenUtil>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace physx;

namespace {

PxTransform getPxTransformFromIsometry(const Isometry3& T)
{
    auto M = T.matrix();
    return PxTransform(
        PxMat44(
            PxVec3(M(0,0), M(1,0), M(2,0)),
            PxVec3(M(0,1), M(1,1), M(2,1)),
            PxVec3(M(0,2), M(1,2), M(2,2)),
            PxVec3(M(0,3), M(1,3), M(2,3))));
}

Isometry3 getIsometryFromPxTransform(const PxTransform& t)
{
    Isometry3 T;
    PxMat44 m(t);
    T.linear() <<
        m.column0.x, m.column1.x, m.column2.x,
        m.column0.y, m.column1.y, m.column2.y,
        m.column0.z, m.column1.z, m.column2.z;
    T.translation() << m.column3.x, m.column3.y, m.column3.z;
    return T;
}

}

namespace cnoid {

PxContinuousTrackHandler::PxContinuousTrackHandler(PxContinuousTrack* device)
    : device_(device),
      trackLink_(nullptr),
      sprocketLink_(nullptr),
      articulation_(nullptr),
      trackArtLink_(nullptr),
      sprocketArtLink_(nullptr),
      idlerArtLink_(nullptr),
      bodyIndex_(0),
      sprocketMimicJoint_(nullptr),
      idlerMimicJoint_(nullptr),
      upperSegmentMimicJoint_(nullptr)
{
    upperSegment_.artLink = nullptr;
    upperSegment_.joint = nullptr;
    upperSegment_.jointPosition = 0.0;
    upperSegment_.direction = 1.0;
    upperSegment_.plateShape = nullptr;
    lowerSegment_.artLink = nullptr;
    lowerSegment_.joint = nullptr;
    lowerSegment_.jointPosition = 0.0;
    lowerSegment_.direction = -1.0;
    lowerSegment_.plateShape = nullptr;
}


PxContinuousTrackHandler::~PxContinuousTrackHandler()
{
    // Linear segment links are owned by the articulation, no need to release them
}


bool PxContinuousTrackHandler::initialize(
    Body* body,
    PxPhysics* physics,
    PxArticulationReducedCoordinate* articulation,
    PxArticulationLink* trackArtLink,
    PxArticulationLink* sprocketArtLink,
    PxArticulationLink* idlerArtLink,
    PxMaterial* material,
    int bodyIndex)
{
    bodyIndex_ = bodyIndex;
    articulation_ = articulation;
    trackArtLink_ = trackArtLink;
    sprocketArtLink_ = sprocketArtLink;
    idlerArtLink_ = idlerArtLink;

    trackLink_ = device_->link();
    if(!trackLink_){
        return false;
    }

    // Check that geometry was initialized by clearState
    if(!device_->isGeometryInitialized()){
        return false;
    }

    auto& spec = device_->spec();
    double sprocketRadius = spec.sprocketRadius;
    double idlerRadius = spec.idlerRadius;

    Link* sprocketLink = body->link(spec.sprocketName);
    Link* idlerLink = body->link(spec.idlerName);
    if(!sprocketLink || !idlerLink){
        return false;
    }
    sprocketLink_ = sprocketLink;

    // Add grouser shapes to the wheels
    addGrouserShapesToWheel(physics, sprocketArtLink, sprocketRadius, material);
    addGrouserShapesToWheel(physics, idlerArtLink, idlerRadius, material);

    // Add belt shapes to the wheels
    addBeltShapesToWheel(physics, sprocketArtLink, sprocketRadius,
                         spec.sprocketBeltSegments, material);
    addBeltShapesToWheel(physics, idlerArtLink, idlerRadius,
                         spec.idlerBeltSegments, material);

    // Store wheel grouser info for visualization
    sprocketGrousers_.wheelLink = sprocketArtLink;
    sprocketGrousers_.wheelRadius = sprocketRadius;
    sprocketGrousers_.numGrousers = spec.numGrousersOnSprocket;
    sprocketGrousers_.numPhysicsGrousers = spec.sprocketFullGrousers;
    sprocketGrousers_.angularOffset = 0.0;
    sprocketGrousers_.angleStep = spec.angleStepSprocket;

    idlerGrousers_.wheelLink = idlerArtLink;
    idlerGrousers_.wheelRadius = idlerRadius;
    idlerGrousers_.numGrousers = spec.numGrousersOnIdler;
    idlerGrousers_.numPhysicsGrousers = spec.idlerFullGrousers;
    idlerGrousers_.angularOffset = 0.0;
    idlerGrousers_.angleStep = spec.angleStepIdler;

    // Calculate linear segment mass
    double totalSegmentMass = spec.mass;
    if(totalSegmentMass <= 0.0){
        totalSegmentMass = sprocketLink->mass() + idlerLink->mass();
    }
    double segmentMass = totalSegmentMass / 2.0;

    // Create linear segments using geometry from spec
    bool success = true;
    success &= createLinearSegment(
        physics, articulation, trackArtLink, material,
        upperSegment_, spec.upperSegmentCenter, spec.slideDir,
        spec.wheelDistance, 1.0, segmentMass);
    success &= createLinearSegment(
        physics, articulation, trackArtLink, material,
        lowerSegment_, spec.lowerSegmentCenter, spec.slideDir,
        spec.wheelDistance, -1.0, segmentMass);

    if(!success){
        return false;
    }

    // Set up velocity drive for the lower segment only (master for mimic joints)
    PxArticulationDrive drive;
    drive.driveType = PxArticulationDriveType::eACCELERATION;
    drive.stiffness = 0.0f;
    drive.damping = static_cast<PxReal>(spec.driveDamping);
    drive.maxForce = (spec.driveMaxForce > 0.0)
        ? static_cast<PxReal>(spec.driveMaxForce)
        : PX_MAX_F32;

    if(lowerSegment_.joint){
        lowerSegment_.joint->setDriveParams(PxArticulationAxis::eX, drive);
    }

    // Create mimic joints to synchronize sprocket, idler, and upper segment
    // with the lower segment (master).
    if(lowerSegment_.joint){
        auto sprocketJoint = sprocketArtLink_->getInboundJoint();
        if(sprocketJoint){
            sprocketMimicJoint_ = articulation->createMimicJoint(
                *lowerSegment_.joint, PxArticulationAxis::eX,
                *sprocketJoint, PxArticulationAxis::eTWIST,
                static_cast<PxReal>(spec.effectiveSprocketRadius), 0.0f);
        }

        auto idlerJoint = idlerArtLink_->getInboundJoint();
        if(idlerJoint){
            idlerMimicJoint_ = articulation->createMimicJoint(
                *lowerSegment_.joint, PxArticulationAxis::eX,
                *idlerJoint, PxArticulationAxis::eTWIST,
                static_cast<PxReal>(spec.effectiveIdlerRadius), 0.0f);
        }

        if(upperSegment_.joint){
            upperSegmentMimicJoint_ = articulation->createMimicJoint(
                *lowerSegment_.joint, PxArticulationAxis::eX,
                *upperSegment_.joint, PxArticulationAxis::eX,
                1.0f, 0.0f);
        }
    }

    // maxNumVisibleShoes is already set by clearState
    return true;
}


void PxContinuousTrackHandler::addGrouserShapesToWheel(
    PxPhysics* physics,
    PxArticulationLink* wheelLink,
    double wheelRadius,
    PxMaterial* material)
{
    auto& spec = device_->spec();
    double shoeHeight = spec.grouserHeight;
    double thickness = spec.thickness;
    double width = spec.width;

    // Number of grousers around the wheel
    double circumference = 2.0 * M_PI * (wheelRadius + thickness);
    int numGrousers = static_cast<int>(round(circumference / spec.grouserSpacing));
    if(numGrousers < 4) numGrousers = 4;

    double angleStep = 2.0 * M_PI / numGrousers;

    double shoeThickness = device_->effectiveGrouserThickness();
    PxBoxGeometry grouserGeom(
        static_cast<PxReal>(shoeThickness / 2.0),
        static_cast<PxReal>(width / 2.0),
        static_cast<PxReal>(shoeHeight / 2.0));

    for(int i = 0; i < numGrousers; ++i){
        double angle = M_PI / 2.0 + i * angleStep;
        double r = wheelRadius + thickness + shoeHeight / 2.0;
        double x = r * cos(angle);
        double z = r * sin(angle);

        PxQuat rot(static_cast<PxReal>(M_PI / 2.0 - angle), PxVec3(0.0f, 1.0f, 0.0f));
        PxTransform localPose(PxVec3(static_cast<PxReal>(x), 0.0f, static_cast<PxReal>(z)), rot);

        PxShape* shape = physics->createShape(grouserGeom, *material, true);
        if(shape){
            shape->setLocalPose(localPose);
            PxFilterData filterData;
            filterData.word1 = static_cast<PxU32>(bodyIndex_);
            shape->setSimulationFilterData(filterData);
            wheelLink->attachShape(*shape);
            shape->release();
        }
    }
}


void PxContinuousTrackHandler::addBeltShapesToWheel(
    PxPhysics* physics,
    PxArticulationLink* wheelLink,
    double wheelRadius,
    int numBeltSegments,
    PxMaterial* material)
{
    auto& spec = device_->spec();
    double thickness = spec.thickness;
    double width = spec.width;

    double angleStep = 2.0 * M_PI / numBeltSegments;
    double r_belt = wheelRadius + thickness / 2.0;
    double r_outer = wheelRadius + thickness;
    double chordLength = 2.0 * r_outer * sin(angleStep / 2.0);

    PxBoxGeometry beltGeom(
        static_cast<PxReal>(chordLength / 2.0),
        static_cast<PxReal>(width / 2.0),
        static_cast<PxReal>(thickness / 2.0));

    for(int i = 0; i < numBeltSegments; ++i){
        double angle = M_PI / 2.0 + i * angleStep;
        double x = r_belt * cos(angle);
        double z = r_belt * sin(angle);

        PxQuat rot(static_cast<PxReal>(M_PI / 2.0 - angle),
                   PxVec3(0.0f, 1.0f, 0.0f));
        PxTransform localPose(
            PxVec3(static_cast<PxReal>(x), 0.0f, static_cast<PxReal>(z)), rot);

        PxShape* shape = physics->createShape(beltGeom, *material, true);
        if(shape){
            shape->setLocalPose(localPose);
            PxFilterData filterData;
            filterData.word1 = static_cast<PxU32>(bodyIndex_);
            shape->setSimulationFilterData(filterData);
            wheelLink->attachShape(*shape);
            shape->release();
        }
    }
}


bool PxContinuousTrackHandler::createLinearSegment(
    PxPhysics* physics,
    PxArticulationReducedCoordinate* articulation,
    PxArticulationLink* parentLink,
    PxMaterial* material,
    LinearSegment& segment,
    const Vector3& localOrigin,
    const Vector3& slideAxis,
    double segmentLength,
    double direction,
    double segmentMass)
{
    auto& spec = device_->spec();
    double shoeHeight = spec.grouserHeight;
    double thickness = spec.thickness;
    double width = spec.width;

    segment.localOrigin = localOrigin;
    segment.slideAxis = slideAxis;
    segment.length = segmentLength;
    segment.direction = direction;
    segment.jointPosition = 0.0;

    // Compute segment link position in world frame
    Isometry3 trackT = trackLink_->T();
    Vector3 worldOrigin = trackT * localOrigin;

    // Create the articulation link for the segment
    Isometry3 segmentIsometry;
    segmentIsometry.setIdentity();
    segmentIsometry.linear() = trackT.linear();
    segmentIsometry.translation() = worldOrigin;
    PxTransform segmentPose = getPxTransformFromIsometry(segmentIsometry);
    segment.artLink = articulation->createLink(parentLink, segmentPose);
    if(!segment.artLink){
        return false;
    }

    // Set mass properties
    segment.artLink->setMass(static_cast<PxReal>(segmentMass));
    segment.artLink->setCMassLocalPose(PxTransform(PxIdentity));
    double w = segmentLength;
    double h = width;
    double d = thickness;
    double m12 = segmentMass / 12.0;
    double Ixx = m12 * (h * h + d * d);
    double Iyy = m12 * (w * w + d * d);
    double Izz = m12 * (w * w + h * h);
    segment.artLink->setMassSpaceInertiaTensor(
        PxVec3(static_cast<PxReal>(Ixx),
               static_cast<PxReal>(Iyy),
               static_cast<PxReal>(Izz)));
    segment.artLink->userData = nullptr;

    // Configure the prismatic joint
    segment.joint = segment.artLink->getInboundJoint();
    if(!segment.joint){
        return false;
    }

    segment.joint->setJointType(PxArticulationJointType::ePRISMATIC);

    Quaternion jointFrameRot = Quaternion::FromTwoVectors(Vector3::UnitX(), slideAxis);

    Isometry3 parentJointFrame;
    parentJointFrame.linear() = jointFrameRot.toRotationMatrix();
    parentJointFrame.translation() = localOrigin;

    Isometry3 childJointFrame;
    childJointFrame.linear() = jointFrameRot.toRotationMatrix();
    childJointFrame.translation().setZero();

    segment.joint->setParentPose(getPxTransformFromIsometry(parentJointFrame));
    segment.joint->setChildPose(getPxTransformFromIsometry(childJointFrame));

    segment.joint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eFREE);

    // Add segment plate shape
    double segmentPlateHalfLength = segmentLength / 2.0;
    PxBoxGeometry segmentPlateGeometry(
        static_cast<PxReal>(segmentPlateHalfLength),
        static_cast<PxReal>(width / 2.0),
        static_cast<PxReal>(thickness / 2.0));
    PxTransform segmentPlatePose(PxVec3(0.0f, 0.0f, static_cast<PxReal>(direction * thickness / 2.0)));
    PxShape* segmentPlateShape = physics->createShape(segmentPlateGeometry, *material, true);
    if(segmentPlateShape){
        segmentPlateShape->setLocalPose(segmentPlatePose);
        PxFilterData filterData;
        filterData.word0 = 0;
        filterData.word1 = static_cast<PxU32>(bodyIndex_);
        segmentPlateShape->setSimulationFilterData(filterData);
        segment.artLink->attachShape(*segmentPlateShape);
        segmentPlateShape->release();
        segment.plateShape = segmentPlateShape;
    }

    // Count grousers on this segment (same logic as clearState)
    int totalGrousers = 0;
    {
        double x = segmentLength / 2.0 - spec.grouserSpacing;
        while(x >= -segmentLength / 2.0 - spec.grouserSpacing * 0.01){
            totalGrousers++;
            x -= spec.grouserSpacing;
        }
    }
    segment.numGrousers = totalGrousers;

    // Add grouser shapes
    double grouserDir = (direction > 0) ? 1.0 : -1.0;
    double shoeThickness = device_->effectiveGrouserThickness();
    PxBoxGeometry grouserGeom(
        static_cast<PxReal>(shoeThickness / 2.0),
        static_cast<PxReal>(width / 2.0),
        static_cast<PxReal>(shoeHeight / 2.0));

    for(int i = 0; i < totalGrousers; ++i){
        double x = segmentLength / 2.0 - spec.grouserSpacing - i * spec.grouserSpacing;
        // For the upper segment, move the last grouser (idler end) to the front (sprocket end)
        if(direction > 0 && i == totalGrousers - 1){
            x = segmentLength / 2.0;
        }
        PxTransform localPose(
            PxVec3(static_cast<PxReal>(x), 0.0f,
                   static_cast<PxReal>(grouserDir * (thickness + shoeHeight / 2.0))));

        PxShape* shape = physics->createShape(grouserGeom, *material, true);
        if(shape){
            shape->setLocalPose(localPose);
            PxFilterData filterData;
            filterData.word0 = 0;
            filterData.word1 = static_cast<PxU32>(bodyIndex_);
            shape->setSimulationFilterData(filterData);
            segment.artLink->attachShape(*shape);
            shape->release();
        }
    }

    return true;
}


bool PxContinuousTrackHandler::resetAllJointPositions()
{
    if(!lowerSegment_.joint) return false;

    float pos = lowerSegment_.joint->getJointPosition(PxArticulationAxis::eX);
    float spacing = static_cast<float>(device_->spec().grouserSpacing);

    // Reset all joints to [0, spacing) range
    auto resetToRange = [](float val, float period) -> float {
        float r = fmod(val, period);
        if(r < 0.0f) r += period;
        return r;
    };

    if(pos >= 0.0f && pos < spacing) return false;

    // Reset lower segment (master)
    float lowerNew = resetToRange(pos, spacing);
    lowerSegment_.joint->setJointPosition(PxArticulationAxis::eX, lowerNew);

    // Reset upper segment, preserving mimic constraint error
    if(upperSegment_.joint && upperSegmentMimicJoint_){
        float upperPos = upperSegment_.joint->getJointPosition(PxArticulationAxis::eX);
        float gearRatio = upperSegmentMimicJoint_->getGearRatio();
        float errBefore = pos + gearRatio * upperPos;
        float upperFmod = resetToRange(upperPos, spacing);
        float errAfterFmod = lowerNew + gearRatio * upperFmod;
        float upperNew = upperFmod + (errBefore - errAfterFmod) / gearRatio;
        upperSegment_.joint->setJointPosition(PxArticulationAxis::eX, upperNew);
    } else if(upperSegment_.joint){
        float upperPos = upperSegment_.joint->getJointPosition(PxArticulationAxis::eX);
        upperSegment_.joint->setJointPosition(
            PxArticulationAxis::eX, resetToRange(upperPos, spacing));
    }

    // Reset sprocket, preserving mimic constraint error
    if(sprocketArtLink_){
        auto joint = sprocketArtLink_->getInboundJoint();
        if(joint && sprocketMimicJoint_){
            float wheelPos = joint->getJointPosition(PxArticulationAxis::eTWIST);
            float gearRatio = sprocketMimicJoint_->getGearRatio();
            float errBefore = pos + gearRatio * wheelPos;
            float anglePeriod = spacing
                / static_cast<float>(device_->spec().effectiveSprocketRadius);
            float wheelFmod = resetToRange(wheelPos, anglePeriod);
            float errAfterFmod = lowerNew + gearRatio * wheelFmod;
            float wheelNew = wheelFmod + (errBefore - errAfterFmod) / gearRatio;
            joint->setJointPosition(PxArticulationAxis::eTWIST, wheelNew);
        }
    }

    // Reset idler, preserving mimic constraint error
    if(idlerArtLink_){
        auto joint = idlerArtLink_->getInboundJoint();
        if(joint && idlerMimicJoint_){
            float wheelPos = joint->getJointPosition(PxArticulationAxis::eTWIST);
            float gearRatio = idlerMimicJoint_->getGearRatio();
            float errBefore = pos + gearRatio * wheelPos;
            float anglePeriod = spacing
                / static_cast<float>(device_->spec().effectiveIdlerRadius);
            float wheelFmod = resetToRange(wheelPos, anglePeriod);
            float errAfterFmod = lowerNew + gearRatio * wheelFmod;
            float wheelNew = wheelFmod + (errBefore - errAfterFmod) / gearRatio;
            joint->setJointPosition(PxArticulationAxis::eTWIST, wheelNew);
        }
    }

    return true;
}


void PxContinuousTrackHandler::updateSimulation()
{
    if(!trackLink_) return;

    double dq = sprocketLink_->dq_target();

    // Drive the lower segment only (master for mimic joints)
    // dq_target is the sprocket angular velocity (rad/s), convert to linear segment velocity
    if(lowerSegment_.joint){
        PxReal segmentVelocity = static_cast<PxReal>(
            dq * device_->spec().effectiveSprocketRadius * lowerSegment_.direction);
        lowerSegment_.joint->setDriveVelocity(PxArticulationAxis::eX, segmentVelocity);
    }

    // Synchronized phase reset of all joints
    if(resetAllJointPositions()){
        articulation_->updateKinematic(PxArticulationKinematicFlag::ePOSITION);
    }

    // Compensate segment plate position for artLink sliding
    // Keep the z offset (direction * thickness/2) so plate bottom stays at wheel surface
    if(upperSegment_.plateShape && upperSegment_.joint){
        float pos = upperSegment_.joint->getJointPosition(PxArticulationAxis::eX);
        float zOffset = static_cast<float>(upperSegment_.direction * device_->spec().thickness / 2.0);
        upperSegment_.plateShape->setLocalPose(PxTransform(PxVec3(-pos, 0.0f, zOffset)));
    }
    if(lowerSegment_.plateShape && lowerSegment_.joint){
        float pos = lowerSegment_.joint->getJointPosition(PxArticulationAxis::eX);
        float zOffset = static_cast<float>(lowerSegment_.direction * device_->spec().thickness / 2.0);
        lowerSegment_.plateShape->setLocalPose(PxTransform(PxVec3(-pos, 0.0f, zOffset)));
    }

    // Wake up articulation if there's any velocity command
    if(dq != 0.0 && articulation_->isSleeping()){
        articulation_->wakeUp();
    }
}


void PxContinuousTrackHandler::addSegmentShoePositions(
    const LinearSegment& segment, const Isometry3& trackT_inv)
{
    Isometry3 T_segment = getIsometryFromPxTransform(segment.artLink->getGlobalPose());
    PxU32 nbShapes = segment.artLink->getNbShapes();
    std::vector<PxShape*> shapes(nbShapes);
    segment.artLink->getShapes(shapes.data(), nbShapes);

    // Skip shape 0 (plate), add grouser shapes (converted to track link local frame)
    for(PxU32 i = 1; i < nbShapes; ++i){
        Isometry3 worldPose = T_segment * getIsometryFromPxTransform(shapes[i]->getLocalPose());
        Isometry3 localPose = trackT_inv * worldPose;
        device_->addShoePosition(SE3(localPose));
    }
}


void PxContinuousTrackHandler::addWheelShoePositions(
    const WheelGrouserSet& wheelGrousers,
    bool isOuterPositive,
    const Isometry3& trackT_inv)
{
    Isometry3 wheelT = getIsometryFromPxTransform(wheelGrousers.wheelLink->getGlobalPose());
    PxU32 nbShapes = wheelGrousers.wheelLink->getNbShapes();
    std::vector<PxShape*> shapes(nbShapes);
    wheelGrousers.wheelLink->getShapes(shapes.data(), nbShapes);

    auto joint = wheelGrousers.wheelLink->getInboundJoint();
    float phi = joint ? joint->getJointPosition(PxArticulationAxis::eTWIST) : 0.0f;

    for(int i = 0; i < wheelGrousers.numPhysicsGrousers; ++i){
        double theta = M_PI / 2.0 + i * wheelGrousers.angleStep - static_cast<double>(phi);
        double c = cos(theta);
        if(isOuterPositive ? (c >= -0.01) : (c <= 0.01)){
            Isometry3 worldPose = wheelT * getIsometryFromPxTransform(shapes[i + 1]->getLocalPose());
            Isometry3 localPose = trackT_inv * worldPose;
            device_->addShoePosition(SE3(localPose));
        }
    }
}


void PxContinuousTrackHandler::updateTrackStates()
{
    if(!trackLink_ || device_->numVisibleShoes() <= 0){
        return;
    }

    Isometry3 trackT_inv = trackLink_->T().inverse();

    device_->clearShoePositions();
    device_->shoePositions().reserve(device_->numVisibleShoes());

    if(upperSegment_.artLink){
        addSegmentShoePositions(upperSegment_, trackT_inv);
    }
    if(lowerSegment_.artLink){
        addSegmentShoePositions(lowerSegment_, trackT_inv);
    }
    addWheelShoePositions(sprocketGrousers_, true, trackT_inv);
    addWheelShoePositions(idlerGrousers_, false, trackT_inv);

    device_->notifyStateChange();
}


PxContinuousTrackSimulator::PxContinuousTrackSimulator()
{
}


PxContinuousTrackSimulator::~PxContinuousTrackSimulator()
{
}


int PxContinuousTrackSimulator::setupTrackHandlers(PhysxBody* physxBody)
{
    Body* body = physxBody->body();
    auto simImpl = physxBody->simImpl;
    int count = 0;

    for(auto& device : body->devices<PxContinuousTrack>()){
        Link* trackLink = device->link();
        Link* sprocketLink = body->link(device->sprocketName());
        Link* idlerLink = body->link(device->idlerName());
        if(!trackLink || !sprocketLink || !idlerLink){
            continue;
        }

        auto trackPhysxLink = dynamic_pointer_cast<PhysxArticulationLink>(
            physxBody->physxLinks[trackLink->index()]);
        auto sprocketPhysxLink = dynamic_pointer_cast<PhysxArticulationLink>(
            physxBody->physxLinks[sprocketLink->index()]);
        auto idlerPhysxLink = dynamic_pointer_cast<PhysxArticulationLink>(
            physxBody->physxLinks[idlerLink->index()]);
        if(!trackPhysxLink || !sprocketPhysxLink || !idlerPhysxLink){
            continue;
        }

        // Reset sprocket/idler drive to NONE to prevent conflict with mimic joints
        auto resetWheelDrive = [](PhysxArticulationLink* physxLink) {
            if(physxLink->joint){
                PxArticulationDrive noDrive;
                noDrive.driveType = PxArticulationDriveType::eNONE;
                noDrive.stiffness = 0.0f;
                noDrive.damping = 0.0f;
                noDrive.maxForce = 0.0f;
                physxLink->joint->setDriveParams(physxLink->articulationAxis, noDrive);
            }
            physxLink->controlInputFunc = []{};
        };
        resetWheelDrive(sprocketPhysxLink.get());
        resetWheelDrive(idlerPhysxLink.get());

        int trackMaterialId = trackLink->materialId();
        if(!device->contactMaterialName().empty()){
            trackMaterialId = Material::idOfName(device->contactMaterialName());
        }
        PxMaterial* material = simImpl->getOrCreatePxMaterial(trackMaterialId);

        auto handler = std::make_unique<PxContinuousTrackHandler>(device);
        if(handler->initialize(
               body, simImpl->physics,
               trackPhysxLink->bodyArticulation->articulation,
               trackPhysxLink->articulationLink,
               sprocketPhysxLink->articulationLink,
               idlerPhysxLink->articulationLink,
               material, physxBody->bodyIndex)){
            handlers_.push_back({std::move(handler)});
            ++count;
        }
    }
    return count;
}


void PxContinuousTrackSimulator::initializeTrackStates()
{
    for(auto& entry : handlers_){
        entry.handler->updateTrackStates();
    }
}


void PxContinuousTrackSimulator::updateSimulation()
{
    for(auto& entry : handlers_){
        entry.handler->updateSimulation();
    }
}


void PxContinuousTrackSimulator::updateTrackStates()
{
    for(auto& entry : handlers_){
        entry.handler->updateTrackStates();
    }
}


bool PxContinuousTrackSimulator::empty() const
{
    return handlers_.empty();
}

}
