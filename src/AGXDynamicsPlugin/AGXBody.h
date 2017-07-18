#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H

#include <cnoid/SimulatorItem>
#include "AGXLinkBody.h"
#include <cnoid/BodyItem>
#include <cnoid/MeshExtractor>

namespace cnoid{

class AGXBody;
typedef ref_ptr<AGXBody> AGXBodyPtr;
class AGXLink;
typedef ref_ptr<AGXLink> AGXLinkPtr;
typedef std::vector<AGXLinkPtr> AGXLinkPtrs;
typedef Eigen::Matrix<float, 3, 1> Vertex;

class AGXLink : public Referenced
{
public:
    AGXLink(int index, LinkPtr link);
    AGXLink::AGXLink(LinkPtr link);
    AGXLink::AGXLink(LinkPtr const link, AGXLinkPtr parent, const Vector3& parentOrigin, AGXBodyPtr const agxBody);
    void setParentLink(AGXLinkPtr link);
    int getIndex();
    AGXLinkBodyRef getAGXLinkBody();
    agx::RigidBodyRef  getAGXRigidBody();
    agx::ConstraintRef getAGXConstraint();
    void createLinkBody();
    void setOriginPosition(const Vector3& parentOrigin);
    void createConstraints();
    void setCollision(bool bOn);
    void setTorqueToAGX();
    void setVelocityToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
private:
    int _index;
    Vector3 origin;
    LinkPtr orgLink;
    AGXLinkPtr agxParentLink;
    AGXLinkBodyRef agxLinkBody;
    void createAGXRigidBody();
    void createAGXGeometry();
    void createAGXShape();
    void detectPrimitiveShape(MeshExtractor* extractor, AGXTrimeshDesc& td);
    void createAGXConstraints();
};

class AGXBody :  public SimulationBody
{
public:
    AGXBody(Body& orgBody);
    void initialize();
    void createBody();
    void setCollision(bool bOn);
    void setTorqueToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
    agx::RigidBodyRef getAGXRigidBody(int index);
    agx::ConstraintRef getAGXConstraint(int index);
    void setAGXMaterial(const int& index, const agx::MaterialRef mat);
    int getNumLinks() const;
    void addAGXLink(AGXLinkPtr const agxLink);
private:
    AGXLinkPtrs agxLinks;
};


}

#endif