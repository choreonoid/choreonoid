#include "AGXBodyPart.h"
#include "AGXBody.h"

namespace cnoid {

////////////////////////////////////////////////////////////
// AGXBodyPart
AGXBodyPart::AGXBodyPart()
{
    _hasSelfCollisionGroupName = false;
    _constraints.clear();
}

bool AGXBodyPart::hasSelfCollisionGroupName() const
{
    return _hasSelfCollisionGroupName;
}

std::string AGXBodyPart::getSelfCollisionGroupName() const
{
    return _selfCollisionGroupName;
}

int AGXBodyPart::numAGXConstraints() const
{
    return _constraints.size();
}

agx::ConstraintRef AGXBodyPart::getAGXConstraint(const int& index) const
{
    return  _constraints[index];
}

int AGXBodyPart::numAGXAssemblys() const
{
    return _assemblys.size();
}

agxSDK::AssemblyRef AGXBodyPart::getAGXAssembly(const int & index)
{
    return _assemblys[index];
}

void AGXBodyPart::setSelfCollsionGroupName(const std::string & name)
{
    _selfCollisionGroupName = name;
    _hasSelfCollisionGroupName = true;
}

void AGXBodyPart::addAGXConstraint(agx::ConstraintRef const constraint)
{
    _constraints.push_back(constraint);
}

void AGXBodyPart::addAGXAssembly(agxSDK::Assembly* assembly)
{
    _assemblys.push_back(assembly);
}

void AGXBodyPart::addAGXLinkedStructure(agxSDK::LinkedStructureRef const structure)
{
    _structures.push_back(structure);
}

////////////////////////////////////////////////////////////
// AGXExtraJoint
AGXExtraJoint::AGXExtraJoint(AGXBodyPtr agxBody)
{
    createJoints(agxBody);
}

void AGXExtraJoint::createJoints(AGXBodyPtr agxBody)
{
    BodyPtr const body = agxBody->body();
    const int n = body->numExtraJoints();
    for (int j = 0; j < n; ++j) {
        ExtraJoint& extraJoint = body->extraJoint(j);

        AGXLinkPtr agxLinkPair[2];
        agxLinkPair[0] = agxBody->getAGXLink(extraJoint.link[0]->index());
        agxLinkPair[1] = agxBody->getAGXLink(extraJoint.link[1]->index());
        if (!agxLinkPair[0] || !agxLinkPair[1]) continue;

        Link* const link = extraJoint.link[0];
        const Vector3 p = link->attitude() * extraJoint.point[0] + link->p();
        const Vector3 a = link->attitude() * extraJoint.axis;
        agx::ConstraintRef constraint = nullptr;
        switch (extraJoint.type) {
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
        addAGXConstraint(constraint);
    }
}


////////////////////////////////////////////////////////////
// AGXContinousTrack
AGXContinousTrack::AGXContinousTrack(AGXLinkPtr footLinkStart, AGXBodyPtr body)
{
    std::stringstream ss;
    ss.str("");
    ss << "SelfCollisionContinousTrack" << generateUID() << body->bodyItem()->name() << std::flush;
    setSelfCollsionGroupName(ss.str());
    _feet.clear();
    _chassisLink = footLinkStart->getAGXParentLink();
    _feet.push_back(footLinkStart);
    addFoot(footLinkStart->getOrgLink(), body);
    createTrackConstraint();
}

void AGXContinousTrack::addFoot(LinkPtr link, AGXBodyPtr body)
{
    for (Link* child = link->child(); child; child = child->sibling()) {
        _feet.push_back(body->getAGXLink(child->index()));
        addFoot(child, body);
    }
}

void AGXContinousTrack::createTrackConstraint()
{
    AGXLinkPtr agxFootLinkStart = _feet[0];
    AGXLinkPtr agxFootLinkEnd = _feet[_feet.size() - 1];

    // Connect footLinkStart and footLinkEnd with Hinge
    // Create Hinge with world coordination
    LinkPtr link = agxFootLinkStart->getOrgLink();
    const Vector3& a = link->a();                       // local
    const Vector3 aw = link->attitude() * a;            // world
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
    addAGXConstraint(constraint);

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

    for (int i = 0; i < _feet.size(); ++i) {
        AGXLinkPtr agxLink = _feet[i];
        // Add plane joint
        pd.frameA = AGXObjectFactory::createFrame();
        pd.rigidBodyA = agxLink->getAGXRigidBody();
        agx::PlaneJointRef pj = AGXObjectFactory::createConstraintPlaneJoint(pd);
        addAGXConstraint((agx::ConstraintRef)pj);

        // Enable collision between tracks and the others. Need to contact with wheels.
        agxLink->getAGXGeometry()->removeGroup(agxLink->getSelfCollisionGroupName());
        agxLink->getAGXGeometry()->addGroup(getSelfCollisionGroupName());
    }
}

}