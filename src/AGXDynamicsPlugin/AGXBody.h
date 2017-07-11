#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_BODY_H

#include <cnoid/SimulatorItem>
#include "AGXLinkBody.h"
#include <cnoid/BodyItem>
#include <cnoid/MeshExtractor>

namespace cnoid{

class AGXLink;
typedef ref_ptr<AGXLink> AGXLinkPtr;
typedef Eigen::Matrix<float, 3, 1> Vertex;

class AGXLink : public Referenced
{
public:
    AGXLink(int index, LinkPtr link);
    void setParentLink(AGXLinkPtr link);
    int getIndex();
    AGXLinkBodyRef getAGXLinkBody();
    agx::RigidBodyRef  getAGXRigidBody();
    agx::ConstraintRef getAGXConstraint();
    void createLinkBody();
    void createConstraints();
    void setCollision(bool bOn);
    void setTorqueToAGX();
    void setLinkStateToAGX();
    void setLinkStateToCnoid();
private:
    int _index;
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
    int getNumLinks();
private:
    std::vector<AGXLinkPtr> agxLinks;
};
typedef ref_ptr<AGXBody> AGXBodyPtr;

}

#endif