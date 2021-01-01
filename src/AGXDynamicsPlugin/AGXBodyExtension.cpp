#include "AGXBodyExtension.h"
#include "AGXBody.h"
#include "AGXScene.h"
#include <iostream>

namespace cnoid{

AGXBodyExtension::AGXBodyExtension(AGXBody* agxBody){
    _agxBody = agxBody;
}

AGXBody* AGXBodyExtension::getAGXBody()
{
    return _agxBody;
}

////////////////////////////////////////////////////////////
// AGXExtraJoint
AGXExtraJoint::AGXExtraJoint(AGXBody* agxBody) : AGXBodyExtension(agxBody)
{
    createJoints();
}

void AGXExtraJoint::createJoints()
{
    AGXBody* const agxBody = getAGXBody();
    Body* const body = agxBody->body();
    const int n = body->numExtraJoints();
    for (int j = 0; j < n; ++j) {
        ExtraJoint& extraJoint = body->extraJoint(j);

        AGXLink* agxLinkPair[2];
        agxLinkPair[0] = agxBody->getAGXLink(extraJoint.link(0)->index());
        agxLinkPair[1] = agxBody->getAGXLink(extraJoint.link(1)->index());
        if (!agxLinkPair[0] || !agxLinkPair[1]) continue;

        Link* const link = extraJoint.link(0);
        const Vector3 p = link->R() * extraJoint.point(0) + link->p();
        const Vector3 a = link->R() * extraJoint.axis();
        agx::ConstraintRef constraint = nullptr;
        switch (extraJoint.type()) {
        case ExtraJoint::EJ_PISTON: {
            AGXHingeDesc hd;
            hd.frameAxis = agx::Vec3(a(0), a(1), a(2));
            hd.frameCenter = agx::Vec3(p(0), p(1), p(2));
            hd.rigidBodyA = agxLinkPair[0]->getAGXRigidBody();
            hd.rigidBodyB = agxLinkPair[1]->getAGXRigidBody();
            constraint = AGXObjectFactory::createConstraint(hd);
        }
        case  ExtraJoint::EJ_BALL: {
            AGXBallJointDesc bd;
            bd.framePoint = agx::Vec3(p(0), p(1), p(2));
            bd.rigidBodyA = agxLinkPair[0]->getAGXRigidBody();
            bd.rigidBodyB = agxLinkPair[1]->getAGXRigidBody();
            constraint = AGXObjectFactory::createConstraint(bd);
        }
        default:
            break;
        }
        getAGXBody()->getAGXScene()->getSimulation()->add(constraint);
    }
}


////////////////////////////////////////////////////////////
// AGXContinuousTrack
AGXContinuousTrack::AGXContinuousTrack(AGXLink* footLinkStart, AGXBody* body) : AGXBodyExtension(body)
{
    _feet.clear();
    _chassisLink = footLinkStart->getAGXParentLink();
    _feet.push_back(footLinkStart);
    addFoot(footLinkStart->getOrgLink(), body);
    createTrackConstraint();
}

void AGXContinuousTrack::addFoot(Link* link, AGXBody* body)
{
    for (Link* child = link->child(); child; child = child->sibling()) {
        _feet.push_back(body->getAGXLink(child->index()));
        addFoot(child, body);
    }
}

void AGXContinuousTrack::createTrackConstraint()
{
    AGXLinkPtr agxFootLinkStart = _feet[0];
    AGXLinkPtr agxFootLinkEnd = _feet[_feet.size() - 1];

    // Connect footLinkStart and footLinkEnd with Hinge
    // Create Hinge with world coordination
    LinkPtr link = agxFootLinkStart->getOrgLink();
    const Vector3& a = link->a();                       // local
    const Vector3 aw = link->R() * a;                   // world
    const Vector3& p = link->p();                       // world
    AGXHingeDesc hd;
    hd.frameAxis = agx::Vec3(aw(0), aw(1), aw(2));
    hd.frameCenter = agx::Vec3(p(0), p(1), p(2));
    hd.rigidBodyA = agxFootLinkStart->getAGXRigidBody();
    hd.rigidBodyB = agxFootLinkEnd->getAGXRigidBody();
    hd.motor.enable = false;
    hd.lock.enable = false;
    hd.range.enable = false;
    agx::ConstraintRef constraint = AGXObjectFactory::createConstraint(hd);
    link->setJointType(Link::ROTATIONAL_JOINT);
    agxFootLinkStart->setAGXConstraint(constraint);
    getAGXBody()->getAGXScene()->getSimulation()->add(constraint);

    // Create PlaneJoint to prvent the track falling off
    // Create joint with parent(example:chasis) coordination 
    const Vector3& b = link->b();
    const agx::Vec3 a0 = agx::Vec3(a(0), a(1), a(2));   // Rotate axis of track. local
    const agx::Vec3& b0 = agx::Vec3(b(0), b(1), b(2));  // Vector from parent to footLinkStart
    const agx::Vec3& p0 = (a0 * b0) * a0;             // Middle position of track(projection vector of b0 to a0). local
    agx::Vec3 nx = agx::Vec3::Z_AXIS().cross(a0);       // Check a0 is parallel to Z. X-Y consists plane joint.
    agx::OrthoMatrix3x3 rotation;
    if (nx.normalize() > 1.0e-6) {
        nx.normal();
        rotation.setColumn(0, nx);
        agx::Vec3 ny = a0.cross(nx);
        ny.normal();
        rotation.setColumn(1, ny);
        rotation.setColumn(2, a0);
    }
    agx::AffineMatrix4x4 af(rotation, p0);
    AGXPlaneJointDesc pd;
    pd.frameB = AGXObjectFactory::createFrame();
    pd.frameB->setMatrix(af);
    pd.rigidBodyB = agxFootLinkStart->getAGXParentLink()->getAGXRigidBody();

    // Generate collision group name to disable collision between tracks
    std::stringstream trackCollsionGroupName;
    trackCollsionGroupName.str("");
    trackCollsionGroupName << "SelfCollisionContinuousTrack" << agx::UuidGenerator().generate().str() << std::flush;
    getAGXBody()->addCollisionGroupNameToDisableCollision(trackCollsionGroupName.str());

    for (int i = 0; i < (int)_feet.size(); ++i) {
        AGXLinkPtr agxLink = _feet[i];
        // Add plane joint
        pd.frameA = AGXObjectFactory::createFrame();
        pd.rigidBodyA = agxLink->getAGXRigidBody();
        agx::PlaneJointRef pj = AGXObjectFactory::createConstraintPlaneJoint(pd);
        getAGXBody()->getAGXScene()->getSimulation()->add((agx::ConstraintRef)pj);

        // Force enable collision between other links
        agxLink->getAGXGeometry()->removeGroup(getAGXBody()->getCollisionGroupName());
        // Disable collision between tracks
        agxLink->getAGXGeometry()->addGroup(trackCollsionGroupName.str());
    }
}


}

